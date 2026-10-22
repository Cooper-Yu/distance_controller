# 固定中心整段转弯净空诊断

`tools/route_turn_clearance.py` 只读已记录的完整激光点，检查车身/轮子矩形包络
从零到目标角度的整个旋转过程。它不发布速度、不调整航点、不批准运动。
目的：识别“旋转前后都够宽，但途中车角扫到障碍”的情况，辅助 P13 排查。

使用已停止并闭合的 JSONL 中 `laser_guard` 事件；该行必须含
`points_base_xy`。这些是 guard 使用的完整有限回波，不可用 ICP 配准工具的
裁剪/降采样点云代替。显式选择行号，避免默默选错历史停车位置。

```bash
cd ~/ros2_ws/src/distance_controller
python3 tools/route_turn_clearance.py --audit /absolute/closed/audit.jsonl \
  --line 123 --angle-deg -90
```

负角右转、正角左转。默认净距 2 cm；采样间的包络位移误差最多 1 mm，
该数值是离散计算界限，不包括雷达测距、安装、车身模型或 odom 误差。
输出初始净距、整个过程净距下界、限制回波、最低净距角度及首个低于门槛的
采样角度。`finite_points_preserve_margin` 只针对提供的点；缺测方向、不同高度、
线缆和转向中心漂移未获证明，因此 `motion_authorized/certified_free` 始终 false。
该报告不能单独决定补转、退让方向或直接恢复中断路线。

本地 Ubuntu-22.04 检查：6 unittest、Ruff 检查通过；包含 0/90 度端点均
有余量而中途包络侵入、10001 点密集扫描与下界对照、已有重叠、无效输入和
显式审计行选择。尚无当前真机 P13 全角度验证。

下一真机检查：先在停车状态记录当前完整扫描、TF、odom、身份和画面；检查
预定转角，在有可见墙面的方向对照 raw/filtered 回波。任何建议位移均须重新
核验沿途及终点空间。旧坐标、一次成功旋转或该工具单次通过都不构成安全保证。
