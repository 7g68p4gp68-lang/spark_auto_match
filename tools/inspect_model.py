#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印 YOLO 权重里的类别，用来确认模型是否包含比赛所需类别。

背景：规则里第四类物品是蛋糕(cake)，但机器人上的权重可能是上一版训的（只有 wine）。
缺类别不会报错，只会在比赛里表现为「二层那块永远抓不到」，所以上机前先查一次。

用法（在机器人上，用能 import torch 的那个 python）：
    python3 tools/inspect_model.py                 # 默认查 ~/yolov8n.pt
    python3 tools/inspect_model.py ~/best.pt

优先使用 ultralytics 读取；旧 YOLOv5 权重则回退到 torch。
"""

import argparse
import os
import sys

# 配置里期望能认出来的类别（和 items_config.yaml 的 items 段对应）
EXPECTED = ["bear", "clock", "banana", "cake"]
ALIASES = {"bear": "wild_bear", "teddy bear": "bear", "wine glass": "wine"}


def normalize_names(names):
    if isinstance(names, dict):
        pairs = names.items()
    elif isinstance(names, (list, tuple)):
        pairs = enumerate(names)
    else:
        return None
    return {int(index): ALIASES.get(str(name), str(name)) for index, name in pairs}


def extract_names(ckpt):
    """从各种 yolov5 权重的结构里把 names 挖出来，挖不到返回 None。"""
    # 1) 直接是 nn.Module（部分导出方式）
    names = getattr(ckpt, "names", None)
    if isinstance(names, dict) and names:
        return normalize_names(names)

    if isinstance(ckpt, dict):
        # 2) hub 权重：{'model': DetectMultiBackend/Model, 'names': {...}}
        if isinstance(ckpt.get("names"), dict) and ckpt["names"]:
            return normalize_names(ckpt["names"])
        model = ckpt.get("model")
        for attr in ("names", "module"):
            sub = getattr(model, attr, None)
            if isinstance(sub, dict) and sub:
                return normalize_names(sub)
            sub_names = getattr(sub, "names", None)
            if isinstance(sub_names, dict) and sub_names:
                return normalize_names(sub_names)
    return None


def main():
    ap = argparse.ArgumentParser(description="打印 YOLO 权重的类别名")
    ap.add_argument("weights", nargs="*", default=["~/yolov8n.pt"],
                    help="权重文件路径，默认 ~/yolov8n.pt")
    args = ap.parse_args()

    YOLO = None
    try:
        from ultralytics import YOLO
    except ImportError:
        pass

    torch = None
    try:
        import torch
    except ImportError:
        pass

    if YOLO is None and torch is None:
        sys.exit("当前 python 既没有 ultralytics 也没有 torch；请激活机器人视觉环境")

    all_names = {}
    for path in args.weights:
        full = os.path.expanduser(path)
        if not os.path.isfile(full):
            print("!! 文件不存在: %s" % full)
            continue
        names = None
        if YOLO is not None:
            try:
                names = normalize_names(YOLO(full).names)
            except Exception as exc:
                print("-- Ultralytics 读取失败，尝试旧权重方式 %s: %s" % (full, exc))

        if not names and torch is not None:
            try:
                # weights_only=False 才能读到 names（torch>=2.6 默认 True 会丢自定义对象）
                try:
                    ckpt = torch.load(full, map_location="cpu", weights_only=False)
                except TypeError:
                    ckpt = torch.load(full, map_location="cpu")
                names = extract_names(ckpt)
            except Exception as exc:
                print("!! 读取失败 %s: %s" % (full, exc))

        if not names:
            print("?? %s 里没找到 names，可能是纯 state_dict，跳过" % full)
            continue

        print("\n== %s ==" % full)
        for idx in sorted(names):
            print("   %2d  %s" % (idx, names[idx]))
        print("   共 %d 类: %s" % (len(names), sorted(names.values())))
        all_names.update({str(v): int(k) for k, v in names.items()})

    if not all_names:
        sys.exit("\n没读到任何权重的类别名")

    print("\n=== 规则需要的类别自检 ===")
    missing = []
    for name in EXPECTED:
        mark = "OK " if name in all_names else "缺!"
        if name not in all_names:
            missing.append(name)
        print("   %s %-8s %s" % (mark, name, "" if name in all_names else "（模型里没有这一类）"))

    if missing:
        print("\n结论：模型缺少 %s。" % missing)
        print("  * cake 缺失 → 二层那块蛋糕永远识别不到，需要重训/换权重，")
        print("    或把 items_config.yaml 的 objects_d 改回 wine 先保证能抓三样；")
        print("  * 其它缺失 → 检查权重是不是拿错了版本。")
    else:
        print("\n结论：权重包含四类名称；仍需用 D435 实拍数据验证准确率和速度。")


if __name__ == "__main__":
    main()
