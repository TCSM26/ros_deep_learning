# TCSM Jetson camera nodes

This package ships several Jetson camera nodes. This document focuses on the two
undistorting CSI-camera nodes and, in particular, the new QR-enabled variant.

## `optimized_camera_node_undistort`

The baseline on-board undistortion node. Its pipeline, in a dedicated
high-priority capture thread, is:

1. Initialize the Jetson CSI camera (`videoSource`, zero-copy, GStreamer).
2. Capture frames; auto-reconnect if the stream drops.
3. Optionally cap the publish rate (`publish_rate`, Hz) by dropping frames early.
4. Resize the captured frame down to `output_width` x `output_height`.
5. Undistort the **downsized** image with remap LUTs precomputed from the
   calibration YAML (`calibration_file`, produced by
   `scripts/calibration_npz_to_yaml.py`).
6. Publish:
   - `image_raw` (`sensor_msgs/Image`, bgr8) — the downsized, undistorted image.
   - `image_raw/camera_info` (`sensor_msgs/CameraInfo`) — intrinsics for the
     published (already-undistorted) image, zero distortion.

This node is intentionally minimal: it does no detection.

## `optimized_camera_node_undistort_qr`

A superset of the baseline node. It preserves the **exact** capture / downsize /
undistort / publish behaviour above (same topics, same `CameraInfo`) and adds
local QR detection, decoding and relative-pose estimation. The QR work runs in a
**separate worker thread** so it never stalls image publishing.

The original node and launch file are untouched; this is an additive node.

### QR enable/disable topic

- Subscribes to `qr_enable_topic` (default `/qr_enable`), type
  `std_msgs/Int32`.
- `data == 1` → enable QR detection and QR publishing.
- `data == 0` → disable QR detection; no QR messages are published.
- Default state is set by the `qr_enabled_default` parameter (default `false`,
  i.e. disabled until enabled).

When QR is disabled the only added per-frame cost is one atomic load and a
counter reset — effectively free. When enabled, detection runs on **1 of every
`qr_process_every_n_frames`** captured frames (default `6`) to keep CPU bounded.

### QR detection input image

QR detection runs on the **original full-sized raw camera frame**, captured
*before* the downsize/undistort step — never on the downsized published image.
The full-sized image is *not* published; only the decoded QR info, the relative
pose, and the downsized polygon are published.

Because detection uses the raw (still-distorted) full-resolution frame,
`solvePnP` uses the **full-resolution camera matrix** (the calibration `K`
scaled from the calibration resolution up to the full frame size) together with
the **real distortion coefficients**. See the next section for why this scaling
is required.

### Why the calibration matrix is scaled for QR pose estimation

The camera calibration shipped with this node was estimated on the **downsized**
image (e.g. 420×240), so its intrinsics `fx, fy, cx, cy` are only valid at that
resolution. QR corners, however, are detected on the **full-size** frame (e.g.
1280×720). `solvePnP` requires that the image points (corners) and the camera
intrinsics correspond to the **same image resolution** — mixing full-size
corners with the downsized matrix gives a wrong pose.

So before pose estimation the node scales `K` up to the full frame:

```
scale_x = full_width  / calibration_width
scale_y = full_height / calibration_height

fx_full = fx_calib * scale_x      cx_full = cx_calib * scale_x
fy_full = fy_calib * scale_y      cy_full = cy_calib * scale_y
```

- `calibration_width` / `calibration_height` are launch parameters describing the
  resolution the calibration was done at (the downsized image). If left at `0`
  they fall back to the size stored in the calibration YAML.
- `full_width` / `full_height` are the actual full-size QR-detection frame size,
  taken from the captured frame at runtime.
- **Distortion coefficients are kept unchanged**: they are normalized,
  dimensionless quantities that are invariant under a pure resize. This assumes
  the full frame is a *resized* version of the calibration field of view, not a
  *crop*. (A crop would additionally shift `cx, cy` rather than just scale them.)

This scaled full-resolution matrix is the one passed to `solvePnP` together with
the full-size corners; the downsized calibration matrix is never used for
full-size pose estimation.

### Published QR topics

When QR is enabled and a QR is detected, the node publishes (first valid QR):

| Topic (param)                          | Default                   | Type                          | Contents |
|----------------------------------------|---------------------------|-------------------------------|----------|
| `qr_data_topic`                        | `/qr/data`                | `std_msgs/String`             | decoded QR text/payload |
| `qr_pose_topic`                        | `/qr/pose`                | `geometry_msgs/PoseStamped`   | camera→QR relative pose |
| `qr_corners_topic`                     | `/qr/corners_downsized`   | `geometry_msgs/PolygonStamped`| 4 QR corners scaled to the downsized image |

The detector uses `cv::QRCodeDetector::detectAndDecodeMulti` (multi-code) on
OpenCV >= 4.3, and falls back to single-code `detectAndDecode` on the older
OpenCV shipped with JetPack. Either way it processes detections in order and
publishes the **first** one that yields a valid pose.

### QR relative-pose estimation

- Object points define a QR-centred coordinate frame using the physical QR size
  `qr_size_m` (default `0.10` m). Corner order matches the OpenCV QR detector
  (top-left, top-right, bottom-right, bottom-left):

  ```
  (-s/2,  s/2, 0)   (top-left)
  ( s/2,  s/2, 0)   (top-right)
  ( s/2, -s/2, 0)   (bottom-right)
  (-s/2, -s/2, 0)   (bottom-left)
  ```

- `cv::solvePnP(..., SOLVEPNP_IPPE_SQUARE)` (ideal for a planar square) returns
  the pose in the OpenCV optical frame.
- The pose is then re-expressed in the ROS `camera_link` convention
  (+X forward, +Y left, +Z up), matching the Python `qr_detection` package's
  `transforms.optical_to_link`. The result is published as `PoseStamped` in the
  `camera` frame.

### Full-resolution corners → downsized image coordinates

The detected corners live in full-resolution pixel coordinates. To overlay them
on the downsized published image (on the PC / visualization side), they are
scaled:

```
scaled_x = full_x * output_width  / full_width
scaled_y = full_y * output_height / full_height
```

and published as `PolygonStamped`. Note: the published image is undistorted
while these corners come from the raw (distorted) full frame, so for strongly
distorted lenses the overlay is approximate near the image edges.

### Safety checks

The node guards against: empty frames, missing/invalid camera calibration,
detector exceptions, an invalid QR corner count (not a multiple of 4), and
failed `solvePnP` (that detection is skipped).

## Build

```bash
cd /home/roger/Github/TCSM/Jetson_TCSM
colcon build --packages-select ros_deep_learning
source install/setup.bash
```

## Run (QR variant)

```bash
ros2 launch ros_deep_learning video_source_undistort_qr.ros2.launch
```

The original node is still available via:

```bash
ros2 launch ros_deep_learning video_source_undistort.ros2.launch
```

### Launch parameters (QR-specific)

| Param                       | Default                 |
|-----------------------------|-------------------------|
| `qr_enable_topic`           | `/qr_enable`            |
| `qr_data_topic`             | `/qr/data`              |
| `qr_pose_topic`             | `/qr/pose`              |
| `qr_corners_topic`          | `/qr/corners_downsized` |
| `qr_size_m`                 | `0.10`                  |
| `qr_process_every_n_frames` | `6`                     |
| `qr_enabled_default`        | `false`                 |
| `calibration_width`         | `420` (calib image W; 0 = use YAML) |
| `calibration_height`        | `240` (calib image H; 0 = use YAML) |

All original camera parameters (`input`, `input_width/height`,
`output_width/height`, `publish_rate`, `calibration_file`, `undistort_alpha`,
etc.) are kept.

## Enable / disable QR at runtime

```bash
# enable
ros2 topic pub /qr_enable std_msgs/msg/Int32 "{data: 1}" --once
# disable
ros2 topic pub /qr_enable std_msgs/msg/Int32 "{data: 0}" --once
```

## Inspect QR output

```bash
ros2 topic echo /qr/data
ros2 topic echo /qr/pose
ros2 topic echo /qr/corners_downsized
```
