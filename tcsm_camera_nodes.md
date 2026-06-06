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

### QR detection vs decoding (OpenCV for corners, ZBar for payload)

Detection and decoding are split between two libraries:

- **Detection (corners):** `cv::QRCodeDetector` — `detectMulti` on OpenCV >= 4.3,
  falling back to single-QR `detect` on the older OpenCV shipped with JetPack.
  Detection is geometric and does **not** need OpenCV's QUIRC decoder.
- **Decoding (payload):** **ZBar**. The OpenCV on the Jetson is built *without*
  the QUIRC backend, so `cv::QRCodeDetector` can find the QR quad but never
  decodes (it logs `Library QUIRC is not linked. No decoding is performed.`).
  ZBar is an independent decoder and does the payload reading instead.

Because the two are independent, a QR can produce a valid **pose even when the
payload can't be decoded** (you'll see `/qr/data` empty / "(undecoded)"). The
node processes detections in order and publishes the **first** one with a valid
pose.

**Decode enhancement:** if the whole-frame ZBar pass returns no payload, the node
retries on an upscaled, contrast-enhanced crop of the detected QR region
(`qr_decode_upscale`, `qr_decode_use_clahe`, `qr_decode_use_sharpen`). This
recovers dense/small codes whose modules are too few pixels at native scale. It
runs only on the QR ROI and only when the cheap whole-frame decode failed.

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

### Verticality prior (`qr_vertical_filter`)

`SOLVEPNP_IPPE_SQUARE` returns **two** planar solutions; the node already prefers
the camera-facing one with the lowest reprojection error. Because the QRs are
pasted upright on boxes, a valid QR plane is **perpendicular to the floor**, so
its face normal is **horizontal**. Enabling `qr_vertical_filter`:

- among the IPPE solutions, prefers the one whose normal is horizontal (this
  resolves the flip ambiguity using the prior), and
- **rejects** the QR entirely if even the best solution's normal tilts more than
  `qr_vertical_tol_deg` (default `20°`) off horizontal.

"Horizontal" is defined relative to world-up in the optical frame, derived from
`qr_cam_pitch_deg` (the camera's downward pitch from horizontal; `0` = level).
The filter is **off by default**; enable it (and set `qr_cam_pitch_deg` to match
your mount) when the QRs are known to be upright. It rejects the typical
flipped/noisy poses whose normal points up or down.

A complementary `qr_roll_filter` constrains the **roll**: an upright QR's bottom
edge (the QR's `+X` axis) is parallel to the floor, i.e. horizontal. When
enabled, it rejects poses whose bottom edge tilts more than `qr_roll_tol_deg`
(default `20°`) off horizontal (same world-up as the verticality filter). Use
both together to pin the QR fully upright: vertical plane **and** level edges.

### Full-resolution corners → downsized image coordinates

The detected corners live in full-resolution **raw (distorted)** pixel
coordinates, but the published image is **downsized and undistorted**. To overlay
them correctly the node maps the corners through the *same* transform as the
published image (`qr_undistort_corners`, default `true`):

1. Scale full-res → downsized (still distorted; the undistort remap's input):
   ```
   scaled_x = full_x * output_width  / full_width
   scaled_y = full_y * output_height / full_height
   ```
2. `cv::undistortPoints` with the downsized distortion-model `K` + distortion,
   reprojected through `new_camera_matrix` (the published image's `K`), giving
   pixel coords in the undistorted published image.

The result is published as `PolygonStamped`. With step 2 the overlay lines up
across the whole frame, including the edges. Set `qr_undistort_corners:=false`
to publish the plain linearly-scaled (distorted) corners instead — those are
only approximate near the edges for a distorted lens.

### Safety checks

The node guards against: empty frames, missing/invalid camera calibration,
detector exceptions, an invalid QR corner count (not a multiple of 4), and
failed `solvePnP` (that detection is skipped).

## Build

The QR node decodes payloads with **ZBar**, so install it once on the Jetson:

```bash
sudo apt install libzbar-dev
```

Then:

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
| `qr_decode_upscale`         | `2.0` (ROI upscale for decode retry) |
| `qr_decode_use_clahe`       | `true`                  |
| `qr_decode_use_sharpen`     | `true`                  |
| `qr_undistort_corners`      | `true` (map corners through image undistortion) |
| `qr_vertical_filter`        | `false` (reject/​disambiguate by QR verticality) |
| `qr_vertical_tol_deg`       | `20.0` (max QR-normal tilt off horizontal) |
| `qr_cam_pitch_deg`          | `0.0` (camera downward pitch; defines world-up) |
| `qr_roll_filter`            | `false` (reject QRs whose bottom edge is not level) |
| `qr_roll_tol_deg`           | `20.0` (max bottom-edge tilt off horizontal) |

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
