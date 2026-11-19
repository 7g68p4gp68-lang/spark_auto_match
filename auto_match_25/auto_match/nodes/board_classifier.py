#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分类板识别节点（A 组新增模块）。

职责边界
--------
输入：`/objects`（vision_msgs/Detection2DArray，detect_ros.py 发布）
输出：`/board_mapping`（std_msgs/String，JSON，latched）

    {"mapping": {"5": "Collection_A", "1": "Collection_B", "3": "Collection_C", "2": "Collection_D"},
     "zones": ["Collection_B", "Collection_C", "Collection_D"],
     "stable": true,
     "items": [[1, 120.0], [3, 240.0], [2, 360.0]],
     "stamp": 1234.5}

规则依据（自动任务赛）：
  * 分类板 0.34m x 0.1m，放玩偶/时钟/香蕉三种图案，顺序每局随机 → 从左到右 = B/C/D；
  * 蛋糕不上分类板，固定送 A 区 → 通过 `board.fixed_by_name` 注入，不用改 runtask.py；
  * 板子每局更换，到达分类区后调用 `~capture` 开始新采集；capture_id 用于拒绝旧的 latched 结果。

下游（D 组 runtask.py）只需要：

    from std_msgs.msg import String
    msg = rospy.wait_for_message("/board_mapping", String, timeout=10)
    mapping = {int(k): v for k, v in json.loads(msg.data)["mapping"].items()}

运行
----
    rosrun auto_match board_classifier.py
    rosservice call /board_classifier/capture
    rosrun auto_match board_classifier.py _single_shot:=true   # 出结果就退出，方便台架调试
"""

import json
import os
import sys

import yaml

import rospy
import rospkg
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from vision_msgs.msg import Detection2DArray

# catkin_install_python 生成的可执行包装器不能作为普通模块导入。
# 确保优先从本文件同目录加载纯逻辑 board_mapping.py。
NODE_DIR = os.path.dirname(os.path.realpath(__file__))
if NODE_DIR not in sys.path:
    sys.path.insert(0, NODE_DIR)

from board_mapping import (MappingVoter, DEFAULT_ZONES, find_duplicate_ids,
                           gap_uniform_ratio, normalize_detections)

DEFAULT_BOARD = {
    "enabled": True,
    "zones": list(DEFAULT_ZONES),
    "expect_num": len(DEFAULT_ZONES),
    "stable_frames": 3,
    "x_tolerance": 30,
    "min_gap_ratio": 0.45,
    "include_names": [],
    "ignore_ids": [10, 20, 30, 40, 50, 99],
    "fixed_by_name": {},
    "capture_on_start": False,
    "roi": [],
    "frame_timeout": 5.0,
}


def load_config():
    """读 items_config.yaml；返回 (board 配置, 物品名->ID 表)。找不到就用默认值。"""
    path = None
    try:
        path = os.path.join(rospkg.RosPack().get_path("auto_match"), "config", "items_config.yaml")
    except Exception as exc:  # rospkg 找不到包时（例如在笔记本上试跑）
        rospy.logwarn("找不到 auto_match 包路径，使用默认配置: %s", exc)

    items = {}
    if path and os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf8") as fp:
                content = yaml.safe_load(fp.read()) or {}
        except Exception as exc:
            rospy.logerr("读取 %s 失败，使用默认配置: %s", path, exc)
            return dict(DEFAULT_BOARD), items

        items = content.get("items") or {}
        cfg = dict(DEFAULT_BOARD)
        cfg.update({k: v for k, v in (content.get("board") or {}).items() if v is not None})
        return cfg, items
    return dict(DEFAULT_BOARD), items


def resolve_ids(names, items):
    """把类别名列表翻译成 ID 列表；缺的跳过并告警。"""
    ids = []
    for name in names or []:
        if name in items:
            ids.append(int(items[name]))
        else:
            rospy.logwarn("board 配置里的类别 %r 不在 items 段，已忽略", name)
    return ids


def resolve_fixed(fixed_by_name, items):
    """{类别名: 区域} -> {ID: 区域}"""
    fixed = {}
    for name, zone in (fixed_by_name or {}).items():
        if name in items:
            fixed[int(items[name])] = str(zone)
        else:
            rospy.logwarn("board.fixed_by_name 里的类别 %r 不在 items 段，已忽略", name)
    return fixed


def to_tuples(obj_array, roi=None):
    """Detection2DArray -> [(item_id, center_x, score), ...]"""
    detections = []
    use_roi = bool(roi and len(roi) == 4)
    if use_roi:
        x_min, y_min, x_max, y_max = [float(value) for value in roi]
    for obj in obj_array.detections:
        if not obj.results:
            continue
        center_x = float(obj.bbox.center.x)
        center_y = float(obj.bbox.center.y)
        if use_roi and not (x_min <= center_x <= x_max and y_min <= center_y <= y_max):
            continue
        hypothesis = obj.results[0]
        detections.append((hypothesis.id, center_x, hypothesis.score))
    return detections


class BoardClassifier(object):
    def __init__(self):
        cfg, items = load_config()

        self.enabled = bool(cfg.get("enabled", True))
        self.pub = rospy.Publisher("/board_mapping", String, queue_size=1, latch=True)
        self.capture_id = 0
        self.active = False
        self.capture_srv = rospy.Service("~capture", Trigger, self.capture)
        self.zones = list(cfg["zones"])
        self.frame_timeout = float(cfg["frame_timeout"])
        self.roi = list(cfg.get("roi") or [])
        self.single_shot = bool(rospy.get_param("~single_shot", False))

        if not self.enabled:
            # 配置里关掉时节点空转，方便临时绕过分类板（runtask 会用兜底放置区）
            rospy.logwarn("board.enabled=false，分类板识别已关闭")
            return

        self.include_ids = resolve_ids(cfg.get("include_names"), items) or None
        self.fixed = resolve_fixed(cfg.get("fixed_by_name"), items)

        self.voter = MappingVoter(
            zones=self.zones,
            expect_num=int(cfg["expect_num"]),
            stable_frames=int(cfg["stable_frames"]),
            x_tolerance=float(cfg["x_tolerance"]),
            ignore_ids=cfg["ignore_ids"],
            include_ids=self.include_ids,
            fixed=self.fixed,
            min_gap_ratio=float(cfg["min_gap_ratio"]),
        )

        self.mapping = {}
        self.frames_seen = 0
        self.last_frame_time = rospy.Time.now()
        self.active = bool(cfg.get("capture_on_start", False)) or self.single_shot

        self.sub = rospy.Subscriber("/objects", Detection2DArray, self.objects_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.watchdog)
        rospy.loginfo("board_classifier 就绪：zones=%s 白名单=%s 固定区=%s stable_frames=%s 间距均匀度下限=%.2f ROI=%s active=%s",
                      self.zones, self.include_ids, self.fixed,
                      self.voter.stable_frames, self.voter.min_gap_ratio,
                      self.roi or "全图", self.active)

    def capture(self, _request):
        """开始一次新的分类板采集，旧的 latched 结果由 capture_id/stamp 区分。"""
        if not self.enabled:
            return TriggerResponse(success=False, message="board.enabled=false")
        self.capture_id += 1
        self.voter.reset()
        self.mapping = {}
        self.frames_seen = 0
        self.last_frame_time = rospy.Time.now()
        self.active = True
        rospy.loginfo("开始分类板采集 capture_id=%d", self.capture_id)
        return TriggerResponse(success=True, message=str(self.capture_id))

    # ------------------------------------------------------------------ 回调
    def objects_cb(self, msg):
        self.last_frame_time = rospy.Time.now()
        if not self.active:
            return

        self.frames_seen += 1

        detections = to_tuples(msg, self.roi)
        raw_items = normalize_detections(detections, self.voter.ignore_ids,
                                         self.voter.x_tolerance)
        items = normalize_detections(detections, self.voter.ignore_ids,
                                     self.voter.x_tolerance, self.voter.include_ids)
        if len(raw_items) > len(items):
            dropped = sorted(set(i[0] for i in raw_items) - set(i[0] for i in items))
            # 白名单外的类别被丢掉是正常的（蛋糕/场地标记），但也要能看见，方便排错
            rospy.logwarn_throttle(3.0, "白名单外的检测已忽略: ids=%s（板子上应只有玩偶/时钟/香蕉）", dropped)
        duplicated = find_duplicate_ids(items)
        if duplicated:
            rospy.logwarn_throttle(2.0, "分类板同一图案出现多次，映射可能不准: ids=%s", duplicated)

        mapping, state = self.voter.update(detections)
        if state == "stable":
            self.mapping = mapping
            self.publish(mapping, msg.header.stamp, items)
            rospy.loginfo("分类板识别稳定: %s", mapping)
            self.active = False
            if self.single_shot:
                rospy.loginfo("single_shot=true，退出")
                rospy.signal_shutdown("single_shot done")
        elif state == "incomplete":
            rospy.logwarn_throttle(
                2.0, "分类板目标不足：本帧 %d 个，需要 %d 个，继续等待",
                len(items), self.voter.expect_num)
        elif state == "geometry":
            rospy.logwarn_throttle(
                2.0, "分类板三格间距不均匀(%.2f)，多半是斜视或凑数，本帧不投票: %s",
                gap_uniform_ratio(items, len(self.zones)), items)
        elif state == "extra":
            rospy.logwarn_throttle(
                2.0, "分类板 ROI 内目标过多：本帧 %d 个，预期 %d 个，本帧不投票",
                len(items), self.voter.expect_num)
        elif state == "duplicate":
            rospy.logwarn_throttle(
                2.0, "分类板出现重复类别，本帧不投票: %s", find_duplicate_ids(items))
        elif state == "unstable":
            rospy.loginfo_throttle(2.0, "分类板识别中（未稳定）: %s", items)

    def watchdog(self, _event):
        if not self.active:
            return
        idle = (rospy.Time.now() - self.last_frame_time).to_sec()
        if idle > self.frame_timeout:
            rospy.logwarn_throttle(
                5.0, "/objects 已经 %.1fs 没有新消息，检查 detect_ros 是否在跑", idle)

    # ------------------------------------------------------------------ 发布
    def publish(self, mapping, stamp, items=None):
        payload = {
            "mapping": {str(int(k)): str(v) for k, v in mapping.items()},
            "zones": self.zones,
            "stable": True,
            "capture_id": self.capture_id,
            "items": [[int(i[0]), float(i[1])] for i in (items or [])],
            "stamp": stamp.to_sec() if hasattr(stamp, "to_sec") else 0.0,
        }
        self.pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("board_classifier")
    BoardClassifier()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
