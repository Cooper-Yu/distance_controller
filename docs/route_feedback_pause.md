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

## 2026-10-23 暂停位移的时间对齐

现场162941的暂停anchor对应发零前约.272s(raw)/.352s(filtered)；最新帧回补后
相对旧anchor净位移约5.79/7.10cm。此前直接用4cm比较，把发零前的运动也计入了暂停。
修复使用source stamp而非receipt：仅对anchor→min(当前源时刻,发零时刻)的固定时间段
给出受配置线速度上限约束的位移额度；发零之后时间增长不能继续扩大额度。
实际比较界限是`0.04m + pre_zero_time * bounded_linear_speed`，不是用预测pose发命令。
源anchor年龄必须仍在原ACTIVE的[-.1,.5]s范围，不能扩大失鲜门槛来申请更多额度。
速度额度不超过本次max_speed，且取暂停前最近已观测线速度/命令大小的较大值并受其约束；
已停稳且零命令的anchor、无发布者的停车边界不额外增加额度。

这是对失去反馈期间位移不确定性的有界校核，不能据此声称精确测得了发零后的实际
物理滑行距离。原始转向中心4cm检查独立保留，从原转向起点累计，不修改其基准或阈值。
所有暂停预算、恢复新帧/停稳/身份要求、固定目标和动作截止保持。
`feedback_pause_started.displacement_anchors`记录源时标、位置与固定额度；恢复完成事件
记录各源最终位移/额度，后续可直接审核，不只凭一条错误猜测。

本地22项纯测试含真实raw/filtered快照回放、旧排队帧的限额、不随等待增长、停稳不补额、
异常anchor拒绝及原转向中心4cm回归。真实localhost ROS `lagged_motion`验证6cm旧锚点
回补可在有界额度内恢复；`drift_excess`验证超过界限持续归零并拒绝完成，身份替换仍失败。
实机版本验证尚待，不认定所有停机漂移均属于延迟。