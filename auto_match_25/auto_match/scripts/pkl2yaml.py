#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 config/mark_map.pkl（4x4 变换矩阵）导出成可读的 config/mark_map.yaml。
在机器人上运行一次即可：

    python3 scripts/pkl2yaml.py

导出后 NavigationManager 会优先读取 yaml（可手改、可 diff、可进 Git）。
"""
import os
import pickle

import rospkg
import yaml
import tf.transformations as tft


def main():
    pkg = rospkg.RosPack().get_path("auto_match")
    pkl_path = os.path.join(pkg, "config", "mark_map.pkl")
    yaml_path = os.path.join(pkg, "config", "mark_map.yaml")

    if not os.path.exists(pkl_path):
        print("找不到:", pkl_path)
        return

    with open(pkl_path, "rb") as f:
        data = pickle.load(f)

    out = {}
    for name, mat in data.items():
        try:
            t = tft.translation_from_matrix(mat)
            e = tft.euler_from_matrix(mat)
            out[name] = {
                "x": round(float(t[0]), 3),
                "y": round(float(t[1]), 3),
                "yaw": round(float(e[2]), 3),
            }
        except Exception as exc:
            print("跳过 %s: %s" % (name, exc))

    with open(yaml_path, "w", encoding="utf8") as f:
        yaml.dump(out, f, default_flow_style=False, allow_unicode=True,
                  sort_keys=True)

    print("已导出 %d 个点位到 %s" % (len(out), yaml_path))
    for k, v in out.items():
        print("  %-18s x=%.3f y=%.3f yaw=%.3f" % (k, v["x"], v["y"], v["yaw"]))


if __name__ == "__main__":
    main()
