# 变更日志

## 2026-11-19 · 自动组（B 导航 / D 调度）

### 修复
- **抓取导航点写错（致命）**：`runtask.py` 循环里 `goto_local("Collection_A")` → `goto_local(sorting_name)`。原逻辑每次抓取都跑向 A 收取区，抓不到金砖。
- **local_costmap 障碍/膨胀层未生效**：`local_costmap_params.yaml` 的 `plugins` 缩进到顶层，改为 `local_costmap` 命名空间下。
- **代价地图参数**：`costmap_common_params.yaml` 障碍层启用、`inflation_radius 0.01→0.30`、激光高度 `2.5/4.0→0/2.0`。

### 变更
- **导航模块接入**：`runtask.py` 由 `RobotMoveAction` 切换到 `NavigationManager`（直接调 move_base action，带超时/卡死检测/重试/状态机）。旧类保留可回滚。
- **新增导航文件**：`nodes/nav_manager.py`、`config/nav_params.yaml`、`config/mark_map.yaml`、`config/route_table.yaml`、`scripts/pkl2yaml.py`。
- **防死循环**：分拣点耗尽时 break，避免 IndexError。
- **堆叠补全**：`drop2()` 后递增 `drop_times`，支持 3 层以上。
- **健壮性**：标定文件读取容错、`wait_for_service` 加超时、废弃函数标注。

### 验证（实机，无场地）
- 相机 D435 30Hz、雷达 YDLIDAR G6 10Hz、底盘 odom、机械臂 swiftpro 动作/气泵 —— 全部正常
- `catkin_make` 0 错误
- 导航栈 `map_server` / `amcl` / `move_base` 全部在线

### 遗留（A 组）
- `items_config.yaml` 缺 `cake` 类别，`runtask.py` 的 `CAKE_ID=5` 永不命中 → "蛋糕先放 A"规则无法完成
- 缺 `yolov5` 包、`auto.pt` / `banana.pt` 模型 → `detect_ros` 启动失败

### 待验证
- 以上改动仅语法/逻辑检查，实机行为需等机器人空闲 + 场地验证（导航接入、3 层堆叠、蛋糕优先 A）。
