# 激光覆盖 shadow 诊断（未接生产入口）

这组文件把两个问题分开：已观察到的有限回波是否限制动作，以及尚未观察到的角度缺口
有多大。它不要求每条射线有效才计算有限点约束，但所有结果均为
`enforced=false / executable=false / certified_free=false`，不能把有限点 `clear`
当作完整空间无障碍。原 `RouteLaserGuard`、生产入口和 4 cm odom 漂移保护未修改。

## 接口与边界

- `route_laser_coverage.diagnose(...)`：纯数学。复用原 guard 的全部有限点、候选与实测
  速度扫掠、2 cm 硬距离、3.5 cm 减速带、0.5 s 预测及不超过 1 mm 的采样误差界。
  同时输出所有有限点中最近的模型净距，不以单墙拟合是否成功作前提。
- `CoverageShadowRouteLaserGuard`：仅显式注入 Console 时运行。保留旧严格 guard 的
  `laser_guard` 事件，另外每秒或状态改变时输出 `laser_coverage_shadow`。候选指令原样
  返回；反馈缺失输出 `unavailable`，取消继续传播。它不会创建 raw 订阅。
  可选 `raw_supplier` 仅在 header、数量和扫描几何/时序完全一致时识别过滤产生的缺口。
- `route_laser_shadow_replay.py`：只读本地 sqlite rosbag；不执行 `rclpy.init`，不创建
  节点或发布器。读取 `/scan_filtered`、`/scan`、`/tf_static`、`/odometry/filtered`；
  静态雷达安装 TF 与 150 ms 内 odom 缺一则记 unavailable 并最终 exit 2。
  每帧在 scan-time 坐标系评估显式给出的**反事实候选指令**，并使用该帧附近实测速度；
  这不是当时实际控制指令的完整重演，不证明扫描时新鲜性或停止距离。

缺失类别区分 NaN、正 Inf、负 Inf、低于 range_min、高于 range_max。配对 raw 有限而
filtered 无效时只叫 `filtered_finite`，不擅自称为机器人自身回波。Inf 保留为未知，
既不添加近障点也不替换为最大量程。原始有限点另做约束报告，不删除近点或改路线。

## 缺口量化的含义

连续缺失射线由两侧最近有限射线界定；跨 0/360° 的缺口合并。记录角宽、类别、两侧
实测距离、是否涉及候选/实测动作方向。纯平移只标前半平面相关，任何旋转保守标全方向；
这只是诊断标签，不再用“有一个无效点”短路掉有效点的几何计算。

审计半径 `R = footprint_radius + max(|v_xy|)*0.5 + 0.02 + |sensor_origin_xy|`
包围候选和实测运动的全部模型扫掠及硬距离。以该半径计算缺口的横向孔径
`2R*sin(min(gap_angle,pi)/2)`；它是有限几何尺度，不是隐藏障碍尺寸或距离的保证。
大于半圈的缺口按直径封顶，不能因为三角函数绕回而变小。即使所有射线有效，仍输出
普通有限射线间隔的最大孔径，且仍不认证连续自由空间。

`finite_fraction` 是有效射线比例，不是已证明自由空间比例。
`finite_rays_reaching_audit_radius` 只计实测距离达到审计半径的射线；较近有限回波仍全量
参与障碍约束，不能借两侧远点推断其后方。缺口内障碍距离下界始终为 `null`。
“缺少计划射线扇区”用相邻角步长超过中位数 1.5 倍作诊断标记，未用作运动允许门槛。

未来若把此诊断变成 enforced 策略，必须先确认传感器失效语义、缺口内可假设的最小
障碍截面、遮挡/不反光物体、低矮物体、安装/附件误差与实测刹停距离。仅凭两侧回波
或孔径小于 2 cm 不能证明没有近障。现在没有可执行的缺口放行阈值。

## 已验证来源与历史样本

2026-10-10 真机参数快照：
`training_notes/checkpoint18/robot_control_rosbot_xl/code_lab/runtime_logs/nav2_preflight_20261010_140211_26661/task6_scan_filter_params.yaml`
中的 `LaserScanBoxFilter` 在 base_link 下设置 x=[-0.195,0.165]、y=±0.145、z=±0.5，
`invert=false`。本地 Humble `/opt/ros/humble/include/laser_filters/box_filter.h` 明确把
盒内点对应 range 改为 quiet_NaN。此参数快照不能代替当前真机参数核验。

现有 footprint 为底盘 (±0.170,±0.135) 与轮子 (±0.135,±0.160) 两矩形并集。
历史 TF 的 laser 相对 base_link 为 x=0.02、yaw=pi，模型高度约 0.17345 m。
它是平面近似，不能证明该高度以下/以上障碍可见，也不等于当前真机附件包络。

对历史 `task6_tf_scan_review` bag 的 186 对同 stamp raw/filtered 扫描：
118661 个有限→有限、14397 个正 Inf→正 Inf、782 个有限→NaN、80 个越界保留。
候选 (0,0,0.1 rad/s) 的有限点约束为 125 clear / 61 slowed；有效比例 85.42–90.83%，
每帧最大缺口 5.49–6.99°、审计圈孔径 24.63–31.35 mm。raw 有限点独立评估则 186 帧
全部 blocked，包括过滤区域内的模型重叠。不能因此直接把 raw 接入原 guard，也不能
证明这些点全是自车；该对比用于区分 mask 影响与真实障碍解释。

## 本地验证与用法

WSL Ubuntu-22.04，先 source `/opt/ros/humble/setup.bash`。当前文件未加入 CMake 安装，
不依赖部署，可在源码目录运行：

```bash
python3 -m unittest discover -s test -p 'test_route_laser*.py'
python3 tools/route_laser_shadow_replay.py --confirm-closed \
  --bag /absolute/path/to/bag --candidate 0 0 0.1 --output /tmp/new_shadow.jsonl
```

output 必须是新文件，避免覆盖已有证据。2026-10-22 已验证 37 个 guard/coverage 单测、
Ruff 与上述历史 186 帧只读 replay。输出 `/tmp/coverage_real_20261010_turn_final.jsonl`；
候选 (0.04,0,0) 也完成 186 帧，136 clear / 50 slowed，输出
`/tmp/coverage_real_20261010_forward_final.jsonl`。完整检查入口为
`/tmp/coverage_final_checks.bash`（结果文件已存在，重跑应使用新的 output 文件名）。
单测覆盖缺口不遮挡已知近障、单 NaN 转向量化、跨零缺口、全无效、缺少扫描扇区、
原始配对身份、实测反向速度、真实适配器原样返回指令、陈旧数据与取消传播。
本轮未做新的真机保护验收。


闭包门禁：显式 --confirm-closed、metadata.yaml 和无 SQLite WAL；同 session 存在 result.json 时必须 closed_metadata 为布尔 true。无 result 的标准包仅依赖显式关闭确认，报告 recorder_closed_metadata_verified=false。归档 supervisor.pid 不用于本机进程探测，避免跨主机 PID 复用假拒绝；该门禁不认证来源身份。
