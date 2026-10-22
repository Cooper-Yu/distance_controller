# 动作扫掠需求覆盖：第一片段，仅 shadow

本片段判断候选速度与实测速度会使哪些车身外侧位置更接近机器人；这些位置有哪些
有限回波的采样支持、哪里仍未知、已知近障是否违反原硬距离。所有输出固定
`enforced=false / executable=false / certified_free=false`，包括 `sampled_supported`。
不接控制器、不发布速度、不改路径、终点、book、P01 或返程合同。原 guard 不变。

## 最小 API

`tools/route_swept_coverage.py` 只依赖 numpy 和现有纯数学 guard：

- `ScanEvidence` 必须提供 scan stamp、接收时间、配对 odom stamp、scan-time pose、
  `LaserMount(x,y,z,roll,pitch,yaw)`、epoch、三个 frame 名、范围/角步长，以及每条计划
  射线的雷达角度与有限距离或显式 `None`。不接受缺失 TF、非有限 pose、超过150 ms配对、
  重复射线、将Inf伪造为最大量程。本片段只支持 roll/pitch≈0；非平面安装拒绝。
- `CoverageHistory.add(frame)` 至多保留六帧。epoch、安装、frame改变，时间倒退、
  超过0.5 s反馈间隔、odom大跳变或无效输入都会清空历史。
- `CurrentState` 必须提供当前pose/stamp、接收/评估时间与epoch/frame。
  `assess(current,candidate,measured)` 返回状态、原因、需求点/未知点数量与范围、
  有限点硬约束、历史丢弃原因和模型界限。当前反馈过期或突跳同样拒绝旧覆盖。

实际适配器必须分别提供ROS stamp和单调接收时间；本模块不会猜TF、GID或时钟来源。

## 几何与证据优先级

候选与实测运动分别使用原常body-twist扫掠和0.5 s horizon。当前base_link内建立5 mm
网格，筛出未来到footprint距离减小、且进入硬距离带的点；带宽包括原2 cm硬距离、
网格半对角和原时间采样误差。平移与旋转外扫角点均检查。当前车身内部无需看穿，
但所有已观察到的模型重叠仍参与原硬约束。不是只看车头或取消转弯后方检查。

查询点经过当前base→odom→扫描时base→真实laser mount转换；不能假定雷达居中朝前。
找最近的计划射线，而不是最近有效射线；无效射线本帧不给支持。有限回波之前的支持
半角最多0.5°，并扣除网格半对角的径向余量。另一新鲜帧可提供其自身有限观测，
但不会将本帧无效值改成有效距离。所有保留有限点都送入原select_command；最新近障
不会被旧远回波、wall fit或多数票删除。旧近障最多保守保留0.5 s。

硬约束只作为诊断字段，applied_command仍等于输入候选，不接管速度。

## 明确限界

有限射线不是连续自由空间证明。最多1°角采样、5 mm网格、短期局部场景假设仍是
未通过真机保护验收的可探测模型，不保证间隙细物体、非反光物、雷达高度之外障碍、
附件包络和刹停距离。0.5 m/s、1 rad/s仅限制离线计算规模，不是允许真机使用的速度。
持续未知缺口进入扫掠范围仍为unknown，不增加孔径放行阈值。

### 最小量程圆盘与当前车身的解析相交边界

只对当前车身外的需求点，可额外证明一帧的整个 `range_min` 圆盘严格包含在
**当前** footprint 并集中。圆心按 scan-time base pose、laser mount、current base pose
变换；不是使用旧车身，也不是用两个矩形的整体包围盒。矩形各边把平面分成内外
小矩形，计算圆心到所有外部小矩形闭包的最短距离；只有该距离大于圆半径及保守
浮点舍入余量时证明才成立。该证明为解析距离计算，不用圆周采样。

证明成立意味着圆盘与当前车身外需求集合的交集为空，因而不再让格子径向余量
伸进车内的部分造成下界拒绝。查询点自身位于车内时不享受此规则。证明不成立、
相切、旧 scan pose 使圆盘移出当前车身，或哪怕极小的车外盲区，仍执行原来的
`range_min + grid_half_diagonal` 下界。`range_min` 本身未修改；射线必须有限、
角度门槛、有限回波末端余量以及 occupied 优先均不变。

报告 `minimum_range_body_proofs` 保留当前圆心、到车外距离、原始 range_min、
包含余量、舍入余量及严格包含结果。这是名义平面模型证明，不认证真实安装误差、
附件包络或雷达高度之外障碍，输出仍固定不可执行。

2026-10-22 的历史 bag 中 `range_min=0.15000000596046448`、mount.x=0.02，
相比名义前沿0.17实际多出约5.96 nm；严格判据不能吞掉这部分车外圆盘。
因此该历史回放仍保持全部unknown，不能将“修正后仍unknown”解释为
真实碰撞。新增测试使用有1 mm包含余量的明确合成输入验证支持改善，同时覆盖
相切、纳米/微米车外盲区、旧位姿、矩形凹角、缺射线、重叠/近障及回波末端余量。

## 闭包 bag 只读分析

```bash
source /opt/ros/humble/setup.bash
python3 tools/route_swept_coverage_replay.py \
  --bag /absolute/closed/bag --confirm-closed \
  --candidate 0 0 0.1 --output /tmp/new_swept_report.jsonl
```

复用route_laser_shadow_replay的只读reader；不执行rclpy.init或创建发布器。
必须有metadata；存在recorder result时必须证实关闭，记录的recorder PID仍存活则拒绝。
输出必须新建。调用方仍需确保没有其他写入者，metadata本身不能证明全部进程已停止。
TF或配对odom缺失则unavailable。

历史reader没有GID连续性或真实单调receipt，因此每帧独立、禁用历史累计，明确输出
`epoch_verified=false / history_enabled=false`。header-time surrogate仅用于离线计算，
不证明现场新鲜性。候选是反事实输入，不代表重放实际命令；使用配对实际odom速度。
后续只有绑定连续watcher证据的适配器才能研究现场多帧历史。

## 本地验证

WSL Ubuntu-22.04：`python3 -m pytest -q test/test_route_swept_coverage.py`。
覆盖前进/实测倒退、旋转外扫、偏置/反向安装、scan-time pose、临时缺帧、最新近障、
epoch/TF/frame/stamp/receipt/odom异常、未知不借邻居放行、JSON与活动bag拒读。
未部署、未做新的真机保护测试。

解析包含修正后本地45项测试通过；Ruff check/format检查通过。历史186帧重放
`/tmp/task6_swept_turn_20261022_v3_containment.jsonl` 仍186/186 unknown，包含证明
0/186，包含余量约 `-5.9604645e-9 m`。逐帧需求点数、unknown点数和有限障碍约束
与v2一致；比较结果在 `/tmp/task6_swept_containment_comparison.json`。
这证实本修正未把原缺射线或微小车外盲区放行，而非新真机验收。


闭包门禁：显式 --confirm-closed、metadata.yaml 和无 SQLite WAL；同 session 存在 result.json 时必须 closed_metadata 为布尔 true。无 result 的标准包仅依赖显式关闭确认，报告 recorder_closed_metadata_verified=false。归档 supervisor.pid 不用于本机进程探测，避免跨主机 PID 复用假拒绝；该门禁不认证来源身份。
