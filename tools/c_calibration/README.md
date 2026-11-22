# C 组：新机器人二层蛋糕抓取标定工具

本目录中的脚本用于取相机图像、核对蛋糕顶面四角，以及读取 SwiftPro 控制器回报的真实坐标。它们是标定工具，**尚未完成二层自动抓取**。

## 已确认的现状（2026-11-22）

- `auto_match/nodes/runtask.py` 已有二层 `ArmAction.grasp1()`，并在主流程中优先抓蛋糕，不需要复制旧机器的抓取代码。
- 新机器的 `~/thefile2.txt` 是 2025 年留下的二层像素映射，`grasp1()` 的接触高度写死为 `z=45 mm`。两者均须在本机实测后使用。
- `swiftpro_write_node` 发布的 `/SwiftproState_topic` 是最后一次命令的回显，**不是位置传感器**。曾有命令回显 `(120,0,100)`，但控制器真实位置仍约 `(89,35,34)`，并报告 `E22 unreachable`。
- 新机现有通用 `yolov8n.pt` 不含蛋糕类别。旧机训练的蛋糕权重在新机当前堆叠画面未检出蛋糕，不能直接启动自动抓取。

## 图像中心

在机器人上先启动 ROS 相机，再取一帧：

```bash
source /opt/ros/noetic/setup.bash
python3 capture_ros_frame.py --output /tmp/cake_frame.png
python3 cake_corners.py --image /tmp/cake_frame.png --manual \
  --output /tmp/cake_corners.png
```

在 VNC 图形界面里点真实顶面的四个角，再按 Enter 确认。也可以用 `--points u1 v1 ... u4 v4` 提供人工核对过的四角。自动模式给出的四角只是候选；这次堆叠画面的自动模式未找到可靠四边形。**不要在新机器上用旧机器的 `transform_3d.dat` 或旧坐标。**没有重新测得相机到吸盘口变换前，只使用脚本输出的像素中心，不使用 `--plane-z` 算机械臂坐标。

## 机械臂真实反馈

先停止占用 `/dev/ttyACM0` 的 ROS SwiftPro 驱动。安装 `pyserial` 后，默认命令只读取真实坐标，不移动：

```bash
python3 -m pip install --user pyserial
python3 swiftpro_serial_move.py
```

需要现场人员确认路径净空后，才能指定一个单次目标点，例如：

```bash
python3 swiftpro_serial_move.py --move 140 35 100 --speed 2000
```

脚本拒绝超出保守边界的目标，并在控制器返回 `E22` 或真实反馈没有到位时报告错误。边界只能限制坐标范围，不能判断是否会撞到金砖、支架或人手。串口诊断结束后，再单独启动 ROS 机械臂驱动；不要让两个程序同时打开同一串口。

## 下一步

1. 用顶面四角确认二层蛋糕的像素中心。
2. 在高位、小步移动吸盘口，由现场侧面观察并记录多个“像素中心 → 真实机械臂坐标”样本。
3. 实测二层蛋糕顶面接触高度和吸盘压缩量，再更新二层标定文件和抓取流程。
4. 用当前堆叠画面验证蛋糕识别模型，检出框稳定后再接入 `grasp1()`。
