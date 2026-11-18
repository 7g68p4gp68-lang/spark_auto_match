#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航模块封装 (NavigationManager)
================================================================
负责：点位加载、点到点导航、异常处理(超时/重试)、状态机

对外接口（供 runtask / 其他模块调用）：
    nav = NavigationManager()
    ok  = nav.goto("Collection_B", with_orientation=True)   # 带朝向到点
    ok  = nav.arrive("Sorting_N")                            # 只到位置，不管朝向
    nav.step_go(0.3)                                         # 前进 0.3m
    nav.step_back()                                          # 后退
    nav.state                                                # IDLE/NAVIGATING/REACHED/FAILED

兼容旧接口（便于替换 runtask.py 里的 RobotMoveAction）：
    nav.goto_local(name)      == nav.goto(name, True)
    nav.arriveto_local(name)  == nav.arrive(name)

设计要点：
- 目标直接下发 move_base Action（不再经 /mark_nav topic 中转，可作为重构方向）
- 点位优先从 config/mark_map.yaml 读，否则回退 config/mark_map.pkl（兼容现有）
- 超时、重试次数、到点阈值从 config/nav_params.yaml 读取
- 状态机：IDLE -> NAVIGATING -> REACHED / FAILED
"""

import os
import pickle

import rospy
import rospkg
import actionlib
import yaml
import tf.transformations as tft

from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import common.msg
from common.msg import MoveStraightDistanceAction


STATE_IDLE = "IDLE"
STATE_NAVIGATING = "NAVIGATING"
STATE_REACHED = "REACHED"
STATE_FAILED = "FAILED"


class NavigationManager(object):
    def __init__(self, pkg_name="auto_match"):
        self.pkg_name = pkg_name
        self.pkg_path = rospkg.RosPack().get_path(pkg_name)
        self.config_dir = os.path.join(self.pkg_path, "config")

        # ---- 参数（config/nav_params.yaml，缺省用内置值）----
        self.goto_timeout = 60.0
        self.max_retries = 2
        self.reach_threshold = 0.08
        self._load_params()

        # ---- 状态 ----
        self.state = STATE_IDLE
        self.current_target = None

        # ---- 点位 ----
        self.marks = self._load_marks()

        # ---- move_base ----
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.move_base.wait_for_server(rospy.Duration(20)):
            rospy.logwarn("[NavigationManager] move_base 服务未就绪")

        # ---- 直行 / 后退 action ----
        self.move_straight = actionlib.SimpleActionClient(
            "move_straight", MoveStraightDistanceAction)
        if not self.move_straight.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("[NavigationManager] move_straight 服务未就绪")

        rospy.loginfo("[NavigationManager] ready, 已加载 %d 个点位", len(self.marks))

    # ------------------------------------------------------------------
    # 初始化辅助
    # ------------------------------------------------------------------
    def _load_params(self):
        path = os.path.join(self.config_dir, "nav_params.yaml")
        if not os.path.exists(path):
            return
        try:
            with open(path, "r", encoding="utf8") as f:
                p = yaml.safe_load(f) or {}
            self.goto_timeout = float(p.get("goto_timeout", self.goto_timeout))
            self.max_retries = int(p.get("max_retries", self.max_retries))
            self.reach_threshold = float(p.get("reach_threshold", self.reach_threshold))
        except Exception as e:
            rospy.logwarn("[NavigationManager] 读取 nav_params.yaml 失败: %s", e)

    def _load_marks(self):
        yaml_path = os.path.join(self.config_dir, "mark_map.yaml")
        pkl_path = os.path.join(self.config_dir, "mark_map.pkl")
        try:
            if os.path.exists(yaml_path):
                with open(yaml_path, "r", encoding="utf8") as f:
                    return yaml.safe_load(f) or {}
            if os.path.exists(pkl_path):
                with open(pkl_path, "rb") as f:
                    return pickle.load(f)
        except Exception as e:
            rospy.logerr("[NavigationManager] 加载点位失败: %s", e)
        rospy.logwarn("[NavigationManager] 未找到点位文件")
        return {}

    def _pose_from_mark(self, mark, with_orientation=True):
        """把点位数据统一转成 geometry_msgs/Pose。
        支持两种格式：
          1) dict: {x, y, yaw?}  (yaml)
          2) 4x4 变换矩阵 numpy array (现有 pkl)
        """
        if isinstance(mark, dict):
            x = float(mark["x"])
            y = float(mark["y"])
            if with_orientation:
                yaw = float(mark.get("yaw", 0.0))
                q = tft.quaternion_from_euler(0.0, 0.0, yaw)
            else:
                q = (0.0, 0.0, 0.0, 1.0)
            return Pose(Point(x, y, 0.0), Quaternion(*q))

        # 4x4 变换矩阵
        tran = tft.translation_from_matrix(mark)
        if with_orientation:
            quat = tft.quaternion_from_matrix(mark)
        else:
            quat = (0.0, 0.0, 0.0, 1.0)
        return Pose(Point(*tran), Quaternion(*quat))

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------
    def has_mark(self, name):
        return name in self.marks

    def goto(self, name, with_orientation=True, retries=None, timeout=None):
        return self._navigate(name, with_orientation, retries, timeout)

    def arrive(self, name, retries=None, timeout=None):
        return self._navigate(name, False, retries, timeout)

    # 兼容旧接口
    def goto_local(self, name):
        return self.goto(name, True)

    def arriveto_local(self, name):
        return self.arrive(name)

    def step_go(self, distance):
        """前进 distance 米"""
        try:
            self.move_straight.send_goal_and_wait(
                common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                    const_rot_vel=0.15,
                    move_distance=float(distance),
                ),
                rospy.Duration(5),
            )
        except Exception as e:
            rospy.logwarn("[NavigationManager] step_go 失败: %s", e)
        return True

    def step_back(self, distance=0.1):
        """后退 distance 米"""
        try:
            self.move_straight.send_goal_and_wait(
                common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                    const_rot_vel=-0.1,
                    move_distance=float(distance),
                ),
                rospy.Duration(5),
            )
        except Exception as e:
            rospy.logwarn("[NavigationManager] step_back 失败: %s", e)
        return True

    # ------------------------------------------------------------------
    # 内部：导航 + 状态机 + 超时重试
    # ------------------------------------------------------------------
    def _navigate(self, name, with_orientation, retries, timeout):
        if name not in self.marks:
            rospy.logerr("[NavigationManager] 点位 '%s' 不存在", name)
            self.state = STATE_FAILED
            return False

        retries = self.max_retries if retries is None else int(retries)
        timeout = self.goto_timeout if timeout is None else float(timeout)

        for attempt in range(retries + 1):
            self.state = STATE_NAVIGATING
            self.current_target = name

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self._pose_from_mark(
                self.marks[name], with_orientation)

            rospy.loginfo("[NavigationManager] 前往 '%s' (第 %d/%d 次)",
                          name, attempt + 1, retries + 1)
            self.move_base.send_goal(goal)

            finished = self.move_base.wait_for_result(rospy.Duration(timeout))
            if not finished:
                rospy.logwarn("[NavigationManager] '%s' 超时，取消", name)
                self.move_base.cancel_goal()
                continue

            status = self.move_base.get_state()
            if status == GoalStatus.SUCCEEDED:
                self.state = STATE_REACHED
                rospy.loginfo("[NavigationManager] '%s' 到达", name)
                return True

            rospy.logwarn("[NavigationManager] '%s' 失败(status=%s)，重试", name, status)

        self.state = STATE_FAILED
        rospy.logerr("[NavigationManager] '%s' 重试 %d 次后仍失败", name, retries + 1)
        return False


if __name__ == "__main__":
    rospy.init_node("navigation_manager")
    NavigationManager()
    rospy.spin()
