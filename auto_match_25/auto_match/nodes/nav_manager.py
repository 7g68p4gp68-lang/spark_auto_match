#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航模块封装 (NavigationManager)
================================================================
职责：点位加载、点到点导航、路径路由、异常处理(超时/卡死/重试)、定位健康检查

对外接口：
    nav = NavigationManager()
    nav.goto("Collection_B", with_orientation=True)      # 带朝向到点
    nav.arrive("Sorting_N")                               # 只到位置(TF距离判定)
    nav.goto_route("Collection_B", "Sorting_W", "pickup") # 按路由表先走中间点
    nav.step_go(0.3) / nav.step_back()
    nav.check_localization()                              # (ok, msg)
    nav.force_relocalize(x, y, yaw)                       # 发 /initialpose 重定位
    nav.state                                             # IDLE/NAVIGATING/REACHED/FAILED

兼容旧接口：goto_local(name) / arriveto_local(name)
"""

import os
import math
import pickle

import rospy
import rospkg
import actionlib
import yaml
import tf
import tf.transformations as tft

from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
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

        # ---- 参数（config/nav_params.yaml）----
        self.goto_timeout = 60.0
        self.max_retries = 2
        self.reach_threshold = 0.08
        self.stuck_check_interval = 1.0
        self.stuck_move_threshold = 0.02
        self.stuck_timeout = 8.0
        self.pose_jump_threshold = 0.5
        self._load_params()

        # ---- 状态 ----
        self.state = STATE_IDLE
        self.current_target = None
        self._last_pose = None

        # ---- 数据 ----
        self.marks = self._load_marks()
        self.routes = self._load_routes()

        # ---- ROS 接口 ----
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.move_base.wait_for_server(rospy.Duration(20)):
            rospy.logwarn("[Nav] move_base 服务未就绪")

        self.move_straight = actionlib.SimpleActionClient(
            "move_straight", MoveStraightDistanceAction)
        if not self.move_straight.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("[Nav] move_straight 服务未就绪")

        self.tf_listener = tf.TransformListener()
        self.initialpose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)

        rospy.loginfo("[Nav] ready, 点位 %d 个, 路由段 %d 组",
                      len(self.marks), len(self.routes))

    # ------------------------------------------------------------------
    # 初始化
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
            self.stuck_check_interval = float(p.get("stuck_check_interval", self.stuck_check_interval))
            self.stuck_move_threshold = float(p.get("stuck_move_threshold", self.stuck_move_threshold))
            self.stuck_timeout = float(p.get("stuck_timeout", self.stuck_timeout))
            self.pose_jump_threshold = float(p.get("pose_jump_threshold", self.pose_jump_threshold))
        except Exception as e:
            rospy.logwarn("[Nav] 读取 nav_params.yaml 失败: %s", e)

    def _load_marks(self):
        yaml_path = os.path.join(self.config_dir, "mark_map.yaml")
        pkl_path = os.path.join(self.config_dir, "mark_map.pkl")
        try:
            if os.path.exists(yaml_path):
                with open(yaml_path, "r", encoding="utf8") as f:
                    data = yaml.safe_load(f)
                if data:
                    return data
            if os.path.exists(pkl_path):
                with open(pkl_path, "rb") as f:
                    return pickle.load(f)
        except Exception as e:
            rospy.logerr("[Nav] 加载点位失败: %s", e)
        rospy.logwarn("[Nav] 未找到有效点位文件")
        return {}

    def _load_routes(self):
        path = os.path.join(self.config_dir, "route_table.yaml")
        if not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf8") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            rospy.logwarn("[Nav] 读取 route_table.yaml 失败: %s", e)
            return {}

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    @staticmethod
    def _pose_from_mark(mark, with_orientation=True):
        """点位 -> geometry_msgs/Pose。
        支持 dict{x,y,yaw?}（yaml）与 4x4 变换矩阵（现有 pkl）。"""
        if isinstance(mark, dict):
            x = float(mark["x"])
            y = float(mark["y"])
            if with_orientation:
                yaw = float(mark.get("yaw", 0.0))
                q = tft.quaternion_from_euler(0.0, 0.0, yaw)
            else:
                q = (0.0, 0.0, 0.0, 1.0)
            return Pose(Point(x, y, 0.0), Quaternion(*q))

        tran = tft.translation_from_matrix(mark)
        if with_orientation:
            quat = tft.quaternion_from_matrix(mark)
        else:
            quat = (0.0, 0.0, 0.0, 1.0)
        return Pose(Point(*tran), Quaternion(*quat))

    def get_position(self):
        try:
            (trans, _rot) = self.tf_listener.lookupTransform(
                "map", "base_footprint", rospy.Time(0))
            return (trans[0], trans[1])
        except Exception:
            return None

    def check_localization(self):
        """返回 (ok:bool, msg:str)。导航前后可调用做健康检查。"""
        pos = self.get_position()
        if pos is None:
            return False, "TF map->base_footprint 不可用"
        if self._last_pose is not None:
            d = math.hypot(pos[0] - self._last_pose[0], pos[1] - self._last_pose[1])
            if d > self.pose_jump_threshold:
                self._last_pose = pos
                return False, "位姿跳变 %.2fm" % d
        self._last_pose = pos
        return True, "ok (x=%.2f, y=%.2f)" % pos

    def force_relocalize(self, x, y, yaw):
        """向 AMCL 发一个初始位姿，用于定位丢失后的重定位。"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position = Point(float(x), float(y), 0.0)
        msg.pose.pose.orientation = Quaternion(
            *tft.quaternion_from_euler(0.0, 0.0, float(yaw)))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        rospy.loginfo("[Nav] 已发送重定位 (%.2f, %.2f, %.2f)", x, y, yaw)

    # ------------------------------------------------------------------
    # 对外导航接口
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

    def get_step(self, drop_zone, sorting_name, phase="pickup"):
        key = "to_pickup" if phase == "pickup" else "to_drop"
        return (self.routes.get(key, {}) or {}).get(drop_zone, {}).get(sorting_name)

    def goto_route(self, drop_zone, sorting_name, phase="pickup"):
        """按路由表先到中间 Step，再到目标。返回 (ok, step)。"""
        step = self.get_step(drop_zone, sorting_name, phase)
        if step and step in self.marks:
            if not self.arrive(step):
                rospy.logwarn("[Nav] 中间点 %s 未到达，直接尝试目标", step)
            else:
                rospy.loginfo("[Nav] 经中间点 %s 前往 %s", step, drop_zone)
        return self.goto(drop_zone), step

    def step_go(self, distance):
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
            rospy.logwarn("[Nav] step_go 失败: %s", e)
        return True

    def step_back(self, distance=0.1):
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
            rospy.logwarn("[Nav] step_back 失败: %s", e)
        return True

    # ------------------------------------------------------------------
    # 内部：导航 + 卡死/超时 + 重试
    # ------------------------------------------------------------------
    def _navigate(self, name, with_orientation, retries, timeout):
        if name not in self.marks:
            rospy.logerr("[Nav] 点位 '%s' 不存在", name)
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

            rospy.loginfo("[Nav] 前往 '%s' (第 %d/%d 次)",
                          name, attempt + 1, retries + 1)
            self.move_base.send_goal(goal)

            if self._wait_goal(name, timeout, with_orientation):
                self.state = STATE_REACHED
                rospy.loginfo("[Nav] '%s' 到达", name)
                return True

            rospy.logwarn("[Nav] '%s' 未到达，准备重试", name)

        self.state = STATE_FAILED
        rospy.logerr("[Nav] '%s' 重试 %d 次后仍失败", name, retries + 1)
        return False

    def _wait_goal(self, name, timeout, with_orientation):
        start = rospy.Time.now()
        last_check = start
        last_pos = self.get_position()
        stuck_start = None
        target = self._pose_from_mark(self.marks[name], with_orientation)

        while not rospy.is_shutdown():
            if self.move_base.wait_for_result(rospy.Duration(0.5)):
                status = self.move_base.get_state()
                if status == GoalStatus.SUCCEEDED:
                    return True
                self.move_base.cancel_goal()
                rospy.sleep(0.3)
                return False

            now = rospy.Time.now()

            # arrive 模式：距离足够近即判定到达
            if not with_orientation:
                pos = self.get_position()
                if pos is not None:
                    d = math.hypot(pos[0] - target.position.x,
                                   pos[1] - target.position.y)
                    if d < self.reach_threshold:
                        rospy.loginfo("[Nav] '%s' 距离 %.3fm < %.3fm，判定到达",
                                      name, d, self.reach_threshold)
                        self.move_base.cancel_goal()
                        rospy.sleep(0.3)
                        return True

            # 总超时
            if (now - start).to_sec() > timeout:
                rospy.logwarn("[Nav] '%s' 超时(%.1fs)", name, timeout)
                self.move_base.cancel_goal()
                rospy.sleep(0.3)
                return False

            # 卡死检测
            if (now - last_check).to_sec() >= self.stuck_check_interval:
                pos = self.get_position()
                if pos is not None and last_pos is not None:
                    moved = math.hypot(pos[0] - last_pos[0], pos[1] - last_pos[1])
                    if moved < self.stuck_move_threshold:
                        if stuck_start is None:
                            stuck_start = now
                        elif (now - stuck_start).to_sec() > self.stuck_timeout:
                            rospy.logwarn("[Nav] '%s' 疑似卡死 %.1fs，取消",
                                          name, self.stuck_timeout)
                            self.move_base.cancel_goal()
                            rospy.sleep(0.3)
                            return False
                    else:
                        stuck_start = None
                    last_pos = pos
                last_check = now

        return False


if __name__ == "__main__":
    rospy.init_node("navigation_manager")
    NavigationManager()
    rospy.spin()
