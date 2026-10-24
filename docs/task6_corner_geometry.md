# Task6 P09/P10 纯局部放置

仅新增 `tools/task6_corner_geometry.py` 与独立测试；没有修改现有 runner、路线、P15 策略，
没有 ROS、硬件、部署或 push。输出 `motion_permission=False` 表示本模块不提供执行授权，
并非 runtime 可以忽略其 `BLOCKED/UNKNOWN` 结果。

所有 policy 参数必须显式传入：

```python
CornerPolicy(front_target_m=.09, front_tolerance_m=.005,
             final_speed_cap_mps=.02, max_lane_correction_m=.08,
             clearance_m=.02, lookahead_m=.20, observation_margin_m=.005,
             stable_scans=3, stable_span_s=.3)
```

`select_lane(points_xy, rays, *, nominal_y, laser_origin, policy, evidence_id)`
输入是同一 source scan 经完整 TF 和有界 odom 配对运到**当前 base_link**的平面数据。
`points_xy` 必须包含全部有效观测点，不删除靠近或重叠车体模型的点。
`rays` 保留每条 scheduled ray，格式 `(bearing_rad, valid_bool, projected_range_m_or_None)`；
未知射线不得伪造成远距离。`laser_origin` 必须同样运输，不能默认真实安装在 base 原点。

P09 从前方局部纵向片段寻找两条横向 rail 的自由间隔中心，不用全局墙拟合，
也不把最近的一个右点当墙距目标。最终选择限定在名义终点 ±8cm，名义坐标只是先验。
`nominal_y` 每次从**同一个未重置的原名义点**变换到当前 body；不能每次重置为零累积扩权。
候选完整侧移及前进 strip 均检查 BOXES 外形和全部障碍。前方目标墙限制
`forward_strip_m=min(lookahead, observed_gap-front_target)`，不将目标墙忽略，也不让
超过目标墙的固定长 strip 制造假阻断。

返回值：

- `TARGET_SELECTED`：`target={x:0,y:delta,yaw:0}` 是本 snapshot 的相对目标，
  有 `supported_interval/limiting_points/forward_strip_m`；不是已经到位。
- `APPROACH_SELECTED`：最多2cm向名义方向的局部推进，停车后重新观察；不能记 P09 完成。
  模型重叠仍标为 ambiguous。只有所有近点沿提议路径不恶化且有改善时可提出移开；
  不把模型重叠称为已证接触，也不删除点。运行层必须复查新扫描的真实改善，不能无限循环。
- `BLOCKED/UNKNOWN`：无符合条件的最终通道/部分推进，不输出可接受终点。

碰撞检查保留2cm净距并附加显式观察误差。可见性只检查真正新扫入的平移 volume；
1cm外向栅格检查整 cell 的角区间和最远角点，未知 gap 不补洞，最多12000 cells。
自由径向前缀来自相邻有效 beam 的保守最小量程和角度 cap，依赖静态平面、相邻 beam
之间没有未建模细障碍的实验假设。不是通用自由空间认证，`certified_free=False`。
狭于外形及两侧余量的通道仍会拒绝，不为达成目标缩车体或降低阈值。

`front_command(front_gap_m,cross_error_m,heading_error_rad,*,policy)` 的残差只有
当前**车体前距减9cm**，不接收旧 odom 终点。到 ±5mm 带先所有轴零命令，
前距已经小于目标带时明确 BLOCKED，不自动倒车或宣布到位。
4cm横向/原 heading 保护保留，前进最多2cm/s。

`check_front_motion(points_xy,rays,*,command,measured,laser_origin,policy,evidence_id,horizon_s)`
复查实际候选和实测 twist，`horizon_s` 显式传入 .5–1秒。返回 `SUPPORTED/BLOCKED/UNKNOWN`。
复用既有点云恒 body-twist 预测；mixed wz 完整保留。只对新扫入 volume 要求观测，
不要求全 leading 半球或未运动的 rear margin 全有效；直行后方 invalid 不会自动否决，
前方真正进入的未知区间仍 UNKNOWN。原始/测量运动重叠不以该函数执行逃逸。

`StoppedPlacementGate(policy).add(stamp_ns,gap,*,stopped,heading_error_rad,cross_error_m)`
要求至少3个不同 source stamp、至少.3秒、gap/heading/cross 都在目标带并明确 stopped。
重复帧不推进，源时间回退/同stamp变内容锁存拒绝；超过.5秒的采样空隙清空稳定窗口。
只有 `TARGET_ACCEPTED` 的 `accepted=True`。执行器还须保留独立反馈、唯一 writer、
时限、取消、停后实际 pose 与固定 P01 起点的连续性检查。

验证命令（WSL Ubuntu-22.04，纯 Python，不启动 ROS）：

```bash
python3 test/test_task6_corner_geometry.py
```

10项行为测试通过：双rail中心、全局wall fit失败的局部通道、目标前墙不假阻断、
部分推进非到位、错误方向/近障碍、invalid volume、模糊重叠单调移开、9cm停止、
distinct停止窗口、前方实际短sweep覆盖/障碍/mixed yaw。没有实机效果、三轮全程或停车距离认证。
