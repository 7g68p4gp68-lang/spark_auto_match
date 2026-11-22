#!/usr/bin/env python3
"""Conservative ROS 1 SwiftPro jog control for measuring the suction-cup center.

Only single-axis nudges are allowed. Pump stays off. The displayed measured
position comes from /SwiftproState_topic; commands are never called feedback.
"""

import argparse
import csv
import math
import time
from pathlib import Path

import rospy
from swiftpro.msg import SwiftproState, position, status


class Jogger:
    def __init__(self, log_path, center_xy=None, min_z=-35.0):
        self.latest = None
        self.position_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
        self.pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)
        self.status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)
        self.state_sub = rospy.Subscriber("/SwiftproState_topic", SwiftproState,
                                          self._on_state, queue_size=1)
        self.log_path = Path(log_path)
        self.center_xy = center_xy
        self.min_z = float(min_z)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_path.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.log_file)
        if self.log_file.tell() == 0:
            self.writer.writerow(("timestamp", "command_x", "command_y", "command_z",
                                  "feedback_x", "feedback_y", "feedback_z", "error_mm"))
        self.log_file.flush()
        deadline = time.monotonic() + 5
        while self.latest is None and time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)
        if self.latest is None:
            raise RuntimeError("No /SwiftproState_topic feedback; refusing to move")
        rospy.sleep(0.2)
        self.pump_off()
        self.status_pub.publish(status(1))

    def _on_state(self, msg):
        self.latest = msg

    def measured(self):
        msg = self.latest
        return (float(msg.x), float(msg.y), float(msg.z))

    def pump_off(self):
        self.pump_pub.publish(status(0))

    def show(self, prefix="MEASURED"):
        xyz = self.measured()
        print(f"{prefix}: x={xyz[0]:.1f} y={xyz[1]:.1f} z={xyz[2]:.1f} mm "+
              f"pump={self.latest.pump} motors={self.latest.swiftpro_status}", flush=True)

    def allowed(self, xyz):
        x, y, z = xyz
        return (160 <= x <= 270 and -85 <= y <= 85 and self.min_z <= z <= 160
                and math.hypot(x, y) <= 280)

    def move(self, xyz, home_stage=False):
        if not home_stage and not self.allowed(xyz):
            print("REJECTED: outside conservative calibration bounds "
                  f"(x 160..270, y -85..85, z {self.min_z:g}..160 mm).")
            return False
        if home_stage and not (50 <= xyz[0] <= 270 and -210 <= xyz[1] <= 210
                               and 35 <= xyz[2] <= 160):
            print("REJECTED: invalid home stage")
            return False
        self.pump_off()
        command = position()
        command.x, command.y, command.z = map(float, xyz)
        self.position_pub.publish(command)
        deadline = time.monotonic() + 4.0
        error = float("inf")
        while time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)
            feedback = self.measured()
            error = math.dist(feedback, xyz)
            if error <= 3.0:
                break
        feedback = self.measured()
        self.writer.writerow((time.strftime("%Y-%m-%dT%H:%M:%S"), *xyz,
                              *feedback, round(error, 2)))
        self.log_file.flush()
        print(f"COMMANDED: x={xyz[0]:.1f} y={xyz[1]:.1f} z={xyz[2]:.1f} mm; "
              f"FEEDBACK: x={feedback[0]:.1f} y={feedback[1]:.1f} "
              f"z={feedback[2]:.1f} mm; error={error:.1f} mm", flush=True)
        if error > 3.0:
            print("STOP: feedback did not reach target. Do not keep nudging.")
            return False
        return True

    def home(self):
        x, y, z = self.measured()
        safe_z = max(75.0, z)
        if z < safe_z and not self.move((x, y, safe_z), home_stage=True):
            return False
        if not self.move((130, 0, safe_z), home_stage=True):
            return False
        return self.move((130, 0, 35), home_stage=True)

    def approach(self):
        """Staged, pump-off approach to 35 mm above a floor cube's top."""
        if self.center_xy is None:
            print("No center supplied; restart with --center-x and --center-y")
            return False
        cx, cy = self.center_xy
        if not self.allowed((cx, cy, -15)):
            print("REJECTED: center outside conservative calibration bounds")
            return False
        x, y, z = self.measured()
        if z < 160 and not self.move((x, y, 160), home_stage=True):
            return False
        for pose in ((cx, cy, 160), (cx, cy, 75), (cx, cy, -15)):
            if not self.move(pose, home_stage=pose[2] >= 75):
                return False
        print("At hover: estimated 35 mm above a floor cube. Pump remains OFF.")
        return True

    def close(self):
        self.pump_off()
        self.log_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", default="/tmp/arm_jog_measurements.csv")
    parser.add_argument("--center-x", type=float,
                        help="Provisional marker-frame x from cake_corners.py")
    parser.add_argument("--center-y", type=float,
                        help="Provisional marker-frame y from cake_corners.py")
    parser.add_argument("--min-z", type=float, default=-35.0,
                        help="Lower wrist limit in mm; default -35, hard floor -50")
    args = parser.parse_args()
    if (args.center_x is None) != (args.center_y is None):
        parser.error("--center-x and --center-y must be given together")
    if not -50.0 <= args.min_z <= -15.0:
        parser.error("--min-z must be between -50 and -15 mm")
    rospy.init_node("arm_calibration_jog", anonymous=True)
    center_xy = ((args.center_x, args.center_y)
                 if args.center_x is not None else None)
    jog = Jogger(args.log, center_xy=center_xy, min_z=args.min_z)
    step = 2.0
    print("Pump OFF. Single-axis commands: x+ x- y+ y- z+ z-. "
          "Other commands: step 1|2|5|10, where, watch, approach, home, quit.")
    print(f"Coordinates are millimetres. Wrist lower limit: {args.min_z:g} mm. "
          "Keep hands out of arm workspace.")
    jog.show()
    try:
        while not rospy.is_shutdown():
            try:
                command = input(f"jog [{step:g} mm]> ").strip().lower()
            except EOFError:
                break
            if command in ("quit", "q", "exit"):
                break
            if command == "where":
                jog.show()
                continue
            if command == "watch":
                for _ in range(25):
                    jog.show()
                    rospy.sleep(0.2)
                continue
            if command == "home":
                jog.home()
                continue
            if command == "approach":
                jog.approach()
                continue
            if command.startswith("step "):
                try:
                    requested = float(command.split()[1])
                except (ValueError, IndexError):
                    requested = None
                if requested not in (1.0, 2.0, 5.0, 10.0):
                    print("Step must be 1, 2, 5 or 10 mm")
                else:
                    step = requested
                continue
            if command in ("x+", "x-", "y+", "y-", "z+", "z-"):
                xyz = list(jog.measured())
                axis = "xyz".index(command[0])
                xyz[axis] += step if command[1] == "+" else -step
                if not jog.move(tuple(xyz)):
                    print("Movement halted; inspect arm and feedback before continuing.")
                continue
            print("Unknown command. Use x+/x-/y+/y-/z+/z-, step, where, watch, approach, home, quit.")
    finally:
        jog.close()
        jog.show("FINAL FEEDBACK")
        print("No automatic motion on exit. Use 'home' while path is clear before quitting.")


if __name__ == "__main__":
    main()
