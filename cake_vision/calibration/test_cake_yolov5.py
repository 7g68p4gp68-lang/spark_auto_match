#!/usr/bin/env python3
"""Try the legacy YOLOv5 cake detector on a saved ROS camera frame."""

import argparse

import cv2
import yolov5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    args = parser.parse_args()

    model = yolov5.load(args.model)
    print("classes:", model.model.names)
    model.conf = 0.05
    frame = cv2.imread(args.image)
    if frame is None:
        raise RuntimeError("Cannot open image: " + args.image)
    results = model(frame, size=640)
    print("detections:", len(results.xyxy[0]))
    for row in results.xyxy[0].tolist():
        print("xyxy/conf/class:", [round(float(v), 3) for v in row])


if __name__ == "__main__":
    main()
