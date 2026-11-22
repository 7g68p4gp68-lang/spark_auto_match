#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不驱动任何硬件的上机前检查。"""

import argparse
import math
import os
import pickle
import subprocess
import sys

import yaml


REQUIRED_MARKS = {
    "Classification_area", "Collection_A", "Collection_B", "Collection_C", "Collection_D",
    "Sorting_W", "Sorting_S", "Sorting_E", "Sorting_N", "Sorting_M",
    "Step_NW", "Step_NE", "Step_SW", "Step_SE",
}


def clean_name(value):
    return str(value).strip().strip("\"'").strip()


def package_path(explicit):
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    try:
        return subprocess.check_output(
            ["rospack", "find", "auto_match"], text=True).strip()
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_marks(config_dir):
    yaml_path = os.path.join(config_dir, "mark_map.yaml")
    pkl_path = os.path.join(config_dir, "mark_map.pkl")
    data = None
    source = None
    if os.path.isfile(yaml_path):
        with open(yaml_path, "r", encoding="utf8") as stream:
            data = yaml.safe_load(stream)
        if data:
            source = yaml_path
    if not data and os.path.isfile(pkl_path):
        with open(pkl_path, "rb") as stream:
            data = pickle.load(stream)
        source = pkl_path
    return {clean_name(k): v for k, v in (data or {}).items()}, source


def check_calibration(path):
    with open(path, "r", encoding="utf8") as stream:
        values = [float(v) for v in stream.read().split()]
    return len(values) == 4 and all(math.isfinite(v) for v in values)


def main():
    parser = argparse.ArgumentParser(description="auto_match 真机测试前静态检查（不会移动机器人）")
    parser.add_argument("--package", help="auto_match 包路径；默认用 rospack 查找")
    parser.add_argument("--mode", choices=("vision", "full"), default="full")
    args = parser.parse_args()

    root = package_path(args.package)
    config_dir = os.path.join(root, "config")
    errors, warnings = [], []

    items_path = os.path.join(config_dir, "items_config.yaml")
    try:
        with open(items_path, "r", encoding="utf8") as stream:
            config = yaml.safe_load(stream) or {}
        detect = config.get("detect", {}) or {}
        models = [detect.get("model_auto"), detect.get("model_banana")]
        models = [os.path.expandvars(os.path.expanduser(str(p))) for p in models if p]
        if not models:
            errors.append("items_config.yaml 没有配置模型")
        for model in models:
            if not os.path.isfile(model):
                errors.append("模型不存在: %s" % model)
    except Exception as exc:
        errors.append("无法读取 items_config.yaml: %s" % exc)

    venv_python = os.environ.get(
        "AUTO_MATCH_VENV_PYTHON", os.path.expanduser("~/yolo_venv/bin/python"))
    if not os.path.isfile(venv_python):
        errors.append("视觉虚拟环境 Python 不存在: %s" % venv_python)
    else:
        result = subprocess.run(
            [venv_python, "-c", "import ultralytics,cv2,rospkg,yaml"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode:
            errors.append("视觉环境依赖导入失败: %s" % result.stderr.strip().splitlines()[-1])

    if args.mode == "full":
        for filename in ("thefile.txt", "thefile2.txt"):
            path = os.path.expanduser("~/" + filename)
            try:
                if not check_calibration(path):
                    errors.append("标定文件必须恰好包含 4 个有限数字: %s" % path)
            except Exception as exc:
                errors.append("标定文件不可用 %s: %s" % (path, exc))

        try:
            marks, source = load_marks(config_dir)
            missing = sorted(REQUIRED_MARKS - set(marks))
            if missing:
                errors.append("缺少导航点位: %s" % ", ".join(missing))
            quoted = [k for k in marks if k != clean_name(k)]
            if quoted:
                warnings.append("发现旧式带引号点位名，运行时会自动清洗")
            print("[INFO] 点位文件: %s (%d 个)" % (source or "未找到", len(marks)))
        except Exception as exc:
            errors.append("点位文件不可用: %s" % exc)

    for warning in warnings:
        print("[WARN] " + warning)
    for error in errors:
        print("[FAIL] " + error)
    if errors:
        print("[RESULT] 未通过：禁止启动完整自动任务")
        return 1
    print("[RESULT] %s 检查通过" % ("视觉" if args.mode == "vision" else "完整任务"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
