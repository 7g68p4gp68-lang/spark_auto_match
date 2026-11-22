#!/usr/bin/env python3
"""Check whether a cake model loads and detects the current camera frame."""

import argparse

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    args = parser.parse_args()

    model = YOLO(args.model)
    print("classes:", model.names)
    for result in model.predict(args.image, conf=0.25, verbose=False):
        for box in result.boxes:
            print("class", int(box.cls[0]), "confidence", float(box.conf[0]),
                  "xyxy", [round(float(v), 1) for v in box.xyxy[0]])


if __name__ == "__main__":
    main()
