#!/usr/bin/env python3
"""One-shot converter: calibration_data.npz -> calibration_data.yaml.

The C++ undistort node reads the YAML via cv::FileStorage (no extra deps).
Run once after each new calibration:

    python3 calibration_npz_to_yaml.py \
        ../data/calibration_data.npz ../data/calibration_data.yaml
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_npz', type=Path)
    parser.add_argument('output_yaml', type=Path)
    args = parser.parse_args()

    data = np.load(str(args.input_npz), allow_pickle=True)
    K = np.asarray(data['K'], dtype=np.float64)
    dist = np.asarray(data['dist'], dtype=np.float64).reshape(-1)
    image_size = np.asarray(data['image_size'], dtype=np.int32).reshape(-1)

    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(args.output_yaml), cv2.FILE_STORAGE_WRITE)
    fs.write('K', K)
    fs.write('dist', dist)
    fs.write('image_width', int(image_size[0]))
    fs.write('image_height', int(image_size[1]))
    fs.release()

    print(f'Wrote {args.output_yaml}')
    print(f'  K = {K.tolist()}')
    print(f'  dist = {dist.tolist()}')
    print(f'  calib size = {int(image_size[0])}x{int(image_size[1])}')


if __name__ == '__main__':
    main()
