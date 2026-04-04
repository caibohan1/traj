# circle 模块说明

## 模块用途

这个目录主要放的是无人机轨迹规划与控制相关代码，包含：

- PX4 直连版本
- MAVROS 版本
- 圆轨迹规划器
- 8 字轨迹规划器
- 简化控制器
- 几何控制器

## 当前可执行节点

- `trajectory_planner`：来自 `src/circlecontrol.cpp`
- `geometric_controller`：来自 `src/GeometricController.cpp`
- `eight_planner`：来自 `src/eightcontrol.cpp`
- `controller_node`：来自 `src/Controller.cpp`
- `mavcircle_planner`：来自 `src/Mav_circlecontrol.cpp`
- `maveight_planner`：来自 `src/Mav_eightcontrol.cpp`
- `mavController`：来自 `src/Mav_Controller.cpp`

## 建议阅读顺序

如果要先看“轨迹规划”，建议按下面顺序读：

1. `src/circlecontrol.cpp`
2. `src/eightcontrol.cpp`
3. `src/GeometricController.cpp`
4. `src/Controller.cpp`
5. `CMakeLists.txt`

## 当前代码理解

### 1. `circlecontrol.cpp`

这是圆轨迹规划器主文件，核心入口是 `TrajectoryPlanner::timer_callback()`。

当前代码流大致如下：

1. 节点启动后，创建轨迹发布器、可视化路径发布器和扩展前馈量发布器。
2. 订阅 PX4 状态，只有在飞机同时处于解锁和 Offboard 状态时，轨迹才真正开始。
3. 在未进入 Offboard 之前，`time_step_` 会被冻结在 0，持续发送初始点。
4. 进入轨迹后，按照时间分成 6 个阶段：
   - 纯 Z 轴起飞
   - XY 平面加速
   - XY 平面匀速巡航
   - XY 平面减速
   - 纯 Z 轴降落
   - 落地锁定
5. Z 轴使用五次多项式平滑函数生成。
6. XY 平面先规划角变量 `theta / omega / alpha / j_theta / s_theta`，再解析映射成圆轨迹。
7. 最后发布位置、速度、加速度、jerk、yaw、yawspeed，以及扩展的 snap 和 yawaccel。

### 2. 当前轨迹规划算法

现在这套不是在线优化器，也不是 A*、MPC、minimum-snap 求解器。

它本质上是：

- Z 和 XY 解耦规划
- 用五次多项式做时间标定
- 用解析式参数化圆轨迹
- 解析计算速度、加速度、jerk、snap

也可以概括成：

“分段时间规划 + 解析几何轨迹 + 高阶导数前馈”

### 3. 控制器分成两条线

#### `src/Controller.cpp`

- 更偏简化版姿态控制
- 主要使用位置、速度、加速度、yaw
- 虽然订阅了 `traj_ext`，但没有真正深度利用 snap / yawaccel

#### `src/GeometricController.cpp`

- 是更完整的几何控制器
- 使用了 jerk、snap、yaw rate、yaw acceleration
- 带有更明显的微分平坦性前馈结构

注意：
当前实际运行哪套控制算法，取决于你启动的是 `geometric_controller` 还是 `controller_node`。

### 4. MAVROS 版本说明

- `src/Mav_circlecontrol.cpp` 是 MAVROS 圆轨迹规划器
- `src/Mav_Controller.cpp` 是 MAVROS 控制器
- `src/Mav_eightcontrol.cpp` 文件名像 8 字轨迹，但代码主体目前看起来还是圆轨迹逻辑，不是完整 8 字轨迹

## 目前建议重点关注的函数

如果只想看圆轨迹规划主线，重点看：

- `src/circlecontrol.cpp` 里的 `TrajectoryPlanner::timer_callback()`
- `src/circlecontrol.cpp` 里的 `init_visual_path()`

如果想看轨迹如何被控制器利用，重点看：

- `src/GeometricController.cpp` 里的 `compute_and_publish_control()`
- `src/Controller.cpp` 里的 `compute_and_publish_control()`

## 后续维护约定

从这次开始，后续只要我继续在 `src/circle` 目录做工作，我都会同步维护这份 README，记录：

- 看了哪些文件
- 做了哪些改动
- 当前判断了什么问题
- 还有哪些待确认事项

## 工作记录

### 2026-04-04

- 阅读并分析了 `src/circlecontrol.cpp`
- 阅读并分析了 `src/Controller.cpp`
- 阅读并分析了 `src/GeometricController.cpp`
- 阅读并分析了 `src/Mav_Controller.cpp`
- 阅读并分析了 `src/Mav_circlecontrol.cpp`
- 阅读并分析了 `src/Mav_eightcontrol.cpp`
- 阅读并分析了 `src/eightcontrol.cpp`
- 阅读并分析了 `CMakeLists.txt`
- 确认 `trajectory_planner` 编译自 `src/circlecontrol.cpp`
- 确认 `geometric_controller` 编译自 `src/GeometricController.cpp`
- 确认 `controller_node` 编译自 `src/Controller.cpp`
- 总结了圆轨迹规划器当前的代码流
- 总结了当前轨迹规划的算法类型
- 新建并开始维护本中文 README
