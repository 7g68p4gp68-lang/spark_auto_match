# 夺宝奇兵自动赛 · 自动任务组 · 代码基线

本仓库是从 Spark-H 机器人（自动组借用调试）上拉取的 `auto_match_25` **真实代码基线**，用于版本管理和多人协作。

> 注意：机器上的代码比官方发布的 `auto_match_25.zip` 新很多，已大改（识别换成 YOLOv5、主逻辑重写、增加二层抓取等），但存在半成品和 bug。

## 目录结构

- `auto_match_25/auto_match/` — 主功能包
  - `nodes/` 各节点（含多个历史版本 `runtask1.py`/`runtask_7_16.py` 等，**当前只使用 `runtask.py`**）
  - `config/` `items_config.yaml`、`mark_map.pkl`、地图、`move_base_param/`
  - `launch/` `auto_match.launch`
- `auto_match_25/common/` — 动作 / 消息 / 服务
- `auto_match_25/spark_alignment_mark/` — AR 码对齐节点
- `auto_match_25/spark_move/` — 直行 / 转向控制
- `tools/` — 标定数据（`thefile.txt` / `thefile2.txt`）与桌面工具脚本

## 导航模块（B 负责）

`auto_match/nodes/nav_manager.py` —— `NavigationManager` 封装：

- **接口**：`goto(name, with_orientation=True)` / `arrive(name)` / `goto_route(drop_zone, sorting_name, phase)` / `step_go(dist)` / `step_back()`
- **定位健康**：`check_localization()` / `force_relocalize(x, y, yaw)`
- **兼容旧接口**：`goto_local(name)` / `arriveto_local(name)`，可一行替换 `runtask.py` 里的 `RobotMoveAction`
- **状态机**：`IDLE → NAVIGATING → REACHED / FAILED`
- **异常处理**：超时取消 + 卡死检测 + 自动重试（次数/阈值均可配）
- **点位**：优先读 `config/mark_map.yaml`（可读可 diff），没有则回退 `config/mark_map.pkl`；上机用 `scripts/pkl2yaml.py` 一键导出
- **路由**：`config/route_table.yaml` 把 `runtask.py` 里 Step 点的 if-else 抽成配置
- **参数**：`config/nav_params.yaml`（超时 / 重试 / 卡死 / 定位阈值），上机只改这里

替换方式（`runtask.py`）：

```python
from nav_manager import NavigationManager
self.robot = NavigationManager()   # 原来是 RobotMoveAction()
```

## 已知问题（待完善）

1. 识别已换 YOLOv5，但 `yolov5` 包与 `auto.pt` / `banana.pt` 模型缺失。
2. `runtask.py` 里 `detect_classification_board()` 未接入主流程，`self.mapping` 未初始化 → 运行会崩。
3. `if item_id == (99 or 10 or 20 or 30)` 逻辑错误（只等于 99）。
4. 香蕉放置分支缺失。

## 环境

- 机器人：Spark-H（借用），Ubuntu 20.04 + ROS Noetic
- AI 环境：conda `python3.7.1`（torch 1.11 + tensorflow 2.4.1 + opencv）
- SSH：`spark@10.25.141.8`（已配置免密）
