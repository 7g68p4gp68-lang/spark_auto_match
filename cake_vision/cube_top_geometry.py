#!/usr/bin/env python3
"""Estimate visible cube top faces from aligned D435 depth, without arm motion.

Returns four image corners and their projective diagonal intersection. A partial
or irregular top face is rejected instead of producing a suction target.
"""

import json
from pathlib import Path

import cv2
import numpy as np


def floor_height_m(depth_mm, K):
    depth = depth_mm.astype(np.float32) / 1000.0
    h, w = depth.shape
    v, u = np.mgrid[:h, :w]
    x = (u - K[2]) * depth / K[0]
    y = (v - K[5]) * depth / K[4]
    xyz = np.stack((x, y, depth), axis=-1)
    valid = (depth > 0.2) & (depth < 1.5)
    background = ((u < w * .13) | (u > w * .92) | (v < h * .06)) & (v < h * .88)
    sample = xyz[valid & background]
    if len(sample) < 1000:
        raise ValueError("Too few floor pixels for plane fit")
    for _ in range(5):
        center = np.median(sample, axis=0)
        _, _, vh = np.linalg.svd(sample - center, full_matrices=False)
        normal = vh[-1]
        residual = (sample - center) @ normal
        sample = sample[np.abs(residual) < max(.006, np.quantile(np.abs(residual), .8))]
    center = sample.mean(axis=0)
    _, _, vh = np.linalg.svd(sample - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    height = -np.sum((xyz - center) * normal, axis=-1)
    height[~valid] = np.nan
    floor_error = float(np.nanpercentile(np.abs(height[valid & background]), 95))
    return height, floor_error


def quad_center(points):
    p = np.asarray(points, dtype=np.float64).reshape(4, 2)
    center = p.mean(axis=0)
    angle = np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0])
    p = p[np.argsort(angle)]
    p = np.roll(p, -np.argmin(p[:, 0] + p[:, 1]), axis=0)
    a, b, c, d = p
    along1 = c - a
    along2 = d - b
    den = np.cross(along1, along2)
    if abs(den) < 1:
        raise ValueError("Degenerate top-face quadrilateral")
    t = np.cross(b - a, along2) / den
    return p, a + t * along1


def find_top_faces(depth_mm, K):
    height, floor_error = floor_height_m(depth_mm, K)
    if floor_error > .018:
        raise ValueError("Unstable floor plane: %.1f mm" % (floor_error * 1000))
    found = []
    for level, expected, tolerance in (("lower", .095, .016),
                                       ("upper", .195, .022)):
        mask = np.uint8(np.isfinite(height) &
                        (np.abs(height - expected) < tolerance)) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                np.ones((3, 3), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        for i in range(1, count):
            x, y, w, h, area = [int(v) for v in stats[i]]
            if area < 2200 or w < 70 or h < 65 or w > 170 or h > 160:
                continue
            component = np.uint8(labels[y:y+h, x:x+w] == i) * 255
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            hull = cv2.convexHull(max(contours, key=cv2.contourArea))
            perimeter = cv2.arcLength(hull, True)
            candidates = [cv2.approxPolyDP(hull, epsilon * perimeter, True)
                          for epsilon in (.015, .025, .035, .05)]
            quads = [q for q in candidates if len(q) == 4 and cv2.isContourConvex(q)]
            coverage = area / float(w * h)
            if not quads or coverage < .65:
                continue
            quad = max(quads, key=cv2.contourArea).reshape(4, 2) + [x, y]
            try:
                ordered, center = quad_center(quad)
            except ValueError:
                continue
            quad_area = cv2.contourArea(np.float32(ordered))
            if quad_area < 3500 or area / quad_area < .65:
                continue
            found.append({"level": level, "corners_uv": ordered.round(1).tolist(),
                          "center_uv": center.round(1).tolist(),
                          "pixel_area": area, "coverage": round(coverage, 3),
                          "height_mm": round(float(np.nanmedian(height[y:y+h, x:x+w][component > 0])) * 1000, 1),
                          "bounds": [x, y, x+w, y+h]})
    return found, floor_error


def main():
    root = Path(__file__).resolve().parent
    depth = np.load(root / "depth.npy")
    info = json.loads((root / "info.json").read_text(encoding="utf-8"))
    color = cv2.imread(str(root / "color.png"))
    faces, error = find_top_faces(depth, info["K"])
    for face in faces:
        points = np.rint(face["corners_uv"]).astype(np.int32)
        cv2.polylines(color, [points], True, (0, 255, 0), 2)
        center = tuple(np.rint(face["center_uv"]).astype(int))
        cv2.drawMarker(color, center, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(color, face["level"], center,
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
    cv2.imwrite(str(root / "top_faces.png"), color)
    print(json.dumps({"floor_error_mm_95": round(error * 1000, 2),
                      "faces": faces}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
