# 起点容差：真实C++隔离独立验证

2026-10-25，WSL Ubuntu-22.04，ROS2 Humble。仅新增test/test_start_position_tolerance.py及本报告，未改production、未stage/commit/push、未操作机器人或云端。

## 实际构建与运行

`source /opt/ros/humble/setup.bash && cd /home/cooper/ros2_ws && colcon build --packages-select distance_controller pid_maze_solver --event-handlers console_direct+`：两包成功6.49s，实际编译initial_centering.cpp、链接distance_controller和test_start_verification。

`source /opt/ros/humble/setup.bash && source /home/cooper/ros2_ws/install/setup.bash && cd /home/cooper/ros2_ws/src/distance_controller && ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q test/test_start_position_tolerance.py`：**16 passed in18.81s**。新增test使用实际C++ executable和既有Room本地激光/odom生成器，ROS domain194 localhost，无真实机器人连接。

| 真实C++条件 | 实际首批15/15稳定观测 | 结果 |
|---|---|---|
| 未传新参数，默认5mm；输入8mm残差 | right=.268000 rear=.288000 angle≈0 | START_REFINE，无START_VERIFIED |
| 参数.010；8mm残差 | right=.268000 rear=.288000 angle≈0 | START_VERIFIED，日志within .0100m及.008727rad |
| 参数.010；14mm残差 | right=.274000 rear=.294000 angle≈0 | START_REFINE，无START_VERIFIED |
| 参数.010；8mm残差、heading .020rad | right=.268501 rear=.288111 angle=-.020000 | START_REFINE，无START_VERIFIED |

每例在首个验证/调整决策后终止本地进程组，检查cmd_vel publisher为0。拒绝是“不直接接受这批起点”，不是声称永久不可通过修正。测试未执行完整路线。

原始runtime.log位于 `/tmp/pytest-of-cooper/pytest-38/test_actual_cpp_gate_*/runtime.log`，临时日志可能被pytest后续清理；本报告保留关键数值。

实际C++对.004、.011、.nan、.inf、-.inf参数均非零退出且报参数名；Python runner NaN/inf/界外拒绝，.005/.010实际生成ROS参数及audit值。既有 `python3 -m pytest -q test/test_task6_route_runner.py`：**13 passed in0.04s**。新增文件ruff format/check通过；仅E402针对加载实际tools树按既有项目方式处理。

## 边界

C++只在precise_start使用min(existing centering tolerance,新参数)，航向min(.008726646rad)未放宽；diff未改heading/feedback/4cm或route final5mm/0.5deg。实际heading反例证明新位置授权不同时放宽heading。min更严格既有参数的额外runtime组合未另测；源码审阅可见min。未将mock命令链当作ROS运行，本报告runtime确为实际C++。

源码SHA256：initial_centering.cpp 093530a9f167beef346f7dfd571911bdcb5ca9d0bc7cbf0b9e1a6bf43402cfa1；task6_route_runner.py daa8da1e002a693aa034baf4b72c43914371f2d7e56b054411bd04e5d9d8cb43。若生产源码改变，应重新核对构建与受影响运行。课程PASS/Task6连续验收保持原状态。
