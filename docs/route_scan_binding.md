# Raw / filtered 逐束来源绑定诊断

`tools/route_scan_binding.py` 是独立纯函数，不初始化 ROS、不订阅或发布、不接控制，
不修改现有 guard、阈值或 shadow 文件。返回值始终 `enforced=false`、
`executable=false`、`certified_free=false`，没有运动许可字段。

```python
report = bind_scan_pair(raw, filtered, scan_tf, filter_manifest, epoch)
```

`raw/filtered` 使用 `LaserScan` 消息形状；必须同 header stamp、frame、ray count、
range bounds、全部 angle/time geometry。数量与角度边界允许 float32 字段的微小
表示差异，但 raw/filtered 对应字段必须精确相等。配对不一致直接 ValueError，
不选择附近时间或裁剪到较短数组。

`scan_tf` 使用 `TransformStamped` 形状，parent 必须 `base_link`、child 等于
scan frame；stamp 为零表示调用方提供静态安装，非零必须恰为 scan stamp。
平移和四元数须有限，四元数 norm 距离 1 不超过原几何惯例的 .01 后显式归一化。
所有原始/归一化参数都保留在报告；采用完整三维旋转，不假定雷达朝前。
这不验证 TF 的实时来源或 scan 内部运动补偿。

`epoch` 是调用方提供的非空标识；本函数没有查询 GID 或 freshness 的能力。
`filter_manifest` 必须是有限 JSON，例如历史参数的显式表示：

```json
{"schema":1,"type":"laser_filters/LaserScanBoxFilter","params":{
  "box_frame":"base_link","invert":false,
  "min_x":-0.195,"max_x":0.165,"min_y":-0.145,"max_y":0.145,
  "min_z":-0.5,"max_z":0.5}}
```

这些值只是历史示例，运行时必须另外读取当前配置，不可当成已核验当前值。
报告记录 manifest 的规范 JSON SHA256 和独立副本，
`active_filter_verified/live_freshness_verified` 始终 false。

每束保留 index、laser angle、raw/filtered 值及类别、两者各自有限点的 base xyz。
Inf/NaN 以明确字符串保留，整个结果可 `json.dumps(..., allow_nan=False)`。
量程外有限值仍保留原数值，但不伪造有效点。raw 有限、filtered 缺失时只标
`raw_finite_filtered_missing`；盒内判断与 `invert` 仅说明**所给盒配置按几何会如何处理**，
不能证明现场过滤的实际原因。严格开区间判断与本地 Humble box_filter.h 一致。

所有 raw 有限点都保留，包括盒内、车体外近点和模型重叠。输出的 planar model gap
只引用现有底盘/轮组矩形包络，不是三维接触判定。`self_return_confirmed` 恒 false；
filter box 并不等于物理车身，尤其后沿与角区不能误删。Inf/NaN、被自车遮住的外侧、
回波之后均不因此获得 free 解释。该报告没有覆盖或可前进门槛。

本地 Ubuntu-22.04 验证：

```bash
python3 -m pytest -q test/test_route_scan_binding.py
ruff check tools/route_scan_binding.py test/test_route_scan_binding.py
ruff format --check tools/route_scan_binding.py test/test_route_scan_binding.py
```

10 项测试通过，含 180°/0.02 m TF、盒内车外点、模型重叠、各类缺失、同束值改变、
配对身份/时间/几何拒绝、TF与manifest拒绝、严格边界/invert、hash及输入不变。
没有真实消息回放、当前真机参数核验、控制接入或新硬件测试；不加入 CMake 安装。