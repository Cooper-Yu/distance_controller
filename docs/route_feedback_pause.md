# Task6 路线反馈暂停

本机制只在 `task6_route_runner --laser-log-only` 严格初始化结束、双 odom
实时基线与发布者身份绑定成功后启用。独立 `motion_console`、supervised 会话、
其他控制器继续使用原保护。C++初始化的独立暂停逻辑见 [initial_feedback_pause.md](initial_feedback_pause.md)。

`/odometry/filtered` 与 `/rosbot_xl_base_controller/odom` 的最后有效帧在时间戳与
接收时间均未超过 0.35 s 时可继续使用；观测 scan 对应限值为 0.8 s。
这些主动停车限值早于外层 0.5 s odom / 1 s scan 最终保护。过期或 TF 暂不可用时，
当前 Python 速度发布者先发零，再记录 `feedback_pause_started`，以不高于 20 Hz
重复发零。停车边界没有发布者时只读等待，必须没有其他控制发布者且已观测到零命令；
不会为恢复额外创建发布者。

一次暂停最多 5 s，一条路线累计最多 10 s。恢复要求两路 odom 各自至少三帧不同
时间戳的新鲜停车数据，连续跨度至少 0.3 s，并有至少三帧暂停后生成的新鲜 scan
和对应 TF。scan 确认窗口也必须连续至少 0.3 s，帧间接收/源时间间隔不超过 0.35 s，
确认时最新 scan 源时间与接收时间均不超过 0.35 s。至少需要 20 条有限且在量程内的
ray；暂时不足时保持零命令，重新积累有效窗口。相同时间戳
不能刷新有效接收时间。发布者身份/坐标系变化、无效数值、时间戳倒退或明显未来值、
pose 跳变、暂停期间超过 4 cm 漂移、取消和超时均终止路线。

`feedback_pause_completed` 只在上述条件满足后输出。之后从最新 pose 重新计算原固定
目标，清空 heading PID、速度限幅器、到达保持与 PathTracker 的内部状态。
原始路径点、转向中心与 4 cm 限值、动作截止时间保持不变；暂停不能推进路线点。

纯测试：`python3 -m unittest discover -s test -p test_route_feedback_pause.py`。
`test/verify_route_feedback_pause_ros.py` 使用真实 ROS 发布/订阅与合成传感器，必须在
独立 localhost ROS domain 运行；它不会连接硬件，也不构成实机验收。
