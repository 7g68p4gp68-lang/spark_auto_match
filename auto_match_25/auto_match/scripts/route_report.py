#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印「分拣点 × 放置区」距离矩阵，上机前做路径分析用。

运行（机器人上，需 ROS 环境）：
    python3 scripts/route_report.py

点位坐标从 config/mark_map.yaml（优先）或 mark_map.pkl 读取。
"""
import os
import pickle

import yaml
import rospkg
import tf.transformations as tft


SORTINGS = ["Sorting_W", "Sorting_S", "Sorting_N", "Sorting_E", "Sorting_M"]
ZONES = ["Collection_A", "Collection_B", "Collection_C", "Collection_D"]


def _xy(mark):
    if isinstance(mark, dict):
        return (float(mark["x"]), float(mark["y"]))
    t = tft.translation_from_matrix(mark)
    return (float(t[0]), float(t[1]))


def load_marks(config_dir):
    ypath = os.path.join(config_dir, "mark_map.yaml")
    ppath = os.path.join(config_dir, "mark_map.pkl")
    if os.path.exists(ypath):
        with open(ypath, "r", encoding="utf8") as f:
            data = yaml.safe_load(f)
        if data:
            return data
    if os.path.exists(ppath):
        with open(ppath, "rb") as f:
            return pickle.load(f)
    return {}


def main():
    pkg = rospkg.RosPack().get_path("auto_match")
    marks = load_marks(os.path.join(pkg, "config"))
    print("已加载点位:", ", ".join(sorted(marks.keys())))
    print()

    print("%-12s" % "dist(m)" + "".join("%12s" % z for z in ZONES))
    for s in SORTINGS:
        if s not in marks:
            print("%-12s" % s + "".join("%12s" % "-" for _ in ZONES))
            continue
        sx, sy = _xy(marks[s])
        row = []
        for z in ZONES:
            if z not in marks:
                row.append("%12s" % "-")
            else:
                zx, zy = _xy(marks[z])
                row.append("%12.2f" % ((sx - zx) ** 2 + (sy - zy) ** 2) ** 0.5)
        print("%-12s" % s + "".join(row))

    print("\n提示：可用 nav_manager.plan_pickup_order(origin, sorting_points, counts)")
    print("按这些距离贪心排抓取顺序，减少空跑。")


if __name__ == "__main__":
    main()
