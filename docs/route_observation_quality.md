# 局部观测质量诊断（仅 shadow）

这组支持代码没有接入运动入口，不修改原激光 guard、速度、路线、点位或返程合同。
`calibrated=false / executable=false / enforced=false / certified_free=false` 固定为假，
包括全部射线有效时。它只回答“运动相关方向是否持续缺少有效测量”，不证明空间无障碍。

## API 与判据

- `route_observation_quality.metrics(...)`：输入 scan-time base 方向的
  `(bearing, finite_range_or_None, kind)`、真实 scan 的角步长、候选/实测速度。
  安装 TF、扫描时姿态转换和有效量程筛选由调用方负责，不能省略后假设雷达与车身同向。
- `QualityTracker.observe(timestamp_s,...,epoch)`：连续质量统计。
  使用同一种连续时间基准；真正 runtime 必须在外部继续核验 scan/receipt/TF/odom 新鲜性。
  本工具不是 watchdog，收不到调用时不会自行触发停车。
- 平移相关区域为行进方向前半平面；候选与实测方向取并集。
  超过显式角速度 deadband 的旋转取全方向，但不要求所有射线有效。
- 对每个有限回波赋予一个“有测量的计划角格”（角步长宽度），求相关区域内角度加权比例。
  角格是测量可用性统计，不是激光束之间的自由空间。丢失的整个扫描扇区也计未知，
  不会因剩下的射线全部有限而得到 100%。Inf/NaN/过滤缺口类别保留。
- 最大未知角宽来自计划角格的补集，跨0/360合并；因此不同于旧报告的“有限端点间距”，
  通常少一个计划步长。切勿混用两种阈值。
- 有效率过低或缺口过大连续达到持续时间才标 `persistent_degradation`；
  无有效回波立即标 `unavailable`。持续时间指质量持续恶化，不表示同一个障碍持续存在。
  epoch变化、时间倒退/重复、帧间隔过大、方向明显改变、非法输入均清持续计时。
- 所有近点必须另走原 `select_command` 扫掠约束，质量良好不能覆盖 `blocked`。
  示例速度 deadband 仅用于质量分区，不会删除实测速度或改变原障碍扫掠。

示例参数（**未真机标定，不是执行允许值**）：
有限角度比例0.75、未知缺口10度、持续0.3s、最大帧间隔0.25s、
平移/旋转分区 deadband 0.005m/s / 0.01rad/s、方向切换5度。
参数化是为了离线比较，不允许靠调到全部通过直接开启保护。

## 历史186帧统计

数据：闭合 `task6_tf_scan_review` bag。实际有限点与TF、150ms内odom配对；
候选平移0.04m/s、旋转±0.1rad/s、斜移(0.03,-0.03)m/s，另包括实测速度。
时长约18.56s。缺publisher/receipt历史，持续性只是header时间诊断，
`epoch_verified=false`，不认证真实运行的时钟、连续epoch或停车距离。

| 候选 | 有效角度比例范围 | 最大未知角宽范围 |
|---|---:|---:|
| 前进 |85.09–90.29%|4.49–6.49°|
| 后退 |85.30–92.51%|4.49–6.49°|
| 左移 |81.41–88.90%|4.99–6.49°|
| 右移 |87.79–95.15%|1.50–3.99°|
| 左/右转 |85.30–90.71%|4.99–6.49°|
| 前右斜移 |83.91–90.85%|4.49–6.49°|

0.75/10°示例全部186帧在示例范围内，**不是路线PASS**。
有限点仍要求减速：前50、后162、左50、右50、左转61、右转82、斜移50帧。

比较更严参数，持续条件均0.3s：

| 候选 | 0.85/6° 持续劣化帧数 | 0.90/5° 持续劣化帧数 |
|---|---:|---:|
| 前进 |1|170|
| 后退 |0|116|
| 左移 |13|183|
| 右移 |0|0|
| 左/右转 |2|179|
| 前右斜移 |1|175|

这一单场景不足以选择执行阈值。下一验证要包含正常墙面、真实近障、线缆/遮挡、
突然失去一侧回波、整个scan冻结、轮子运动中odom重置，而非只取正常样本通过率。

## 运行与证据

Ubuntu-22.04 / ROS Humble：

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s test -p test_route_observation_quality.py
python3 tools/route_observation_quality_replay.py \
  --bag /absolute/closed/bag --confirm-closed \
  --output /tmp/new_quality.jsonl \
  --minimum-fraction .85 --maximum-gap-deg 6 --persistence-s .3
```

必须是已关闭bag，禁止现场查活动SQLite。检查metadata/result/owner PID；
历史bag缺少owner审计时仍需操作者确认已关闭。
不初始化ROS、不发布、不覆盖输出；summary写到同名 `.summary.json`。
13单测、Ruff3文件和 `git diff --check` 已通过；
三组186帧均failed_frames=0。证据：
`/tmp/task6_quality_20261022.{jsonl,summary.json}`、
`/tmp/task6_quality85_20261022.{jsonl,summary.json}`、
`/tmp/task6_quality90_20261022.{jsonl,summary.json}`。
复现脚本 `/tmp/task6_quality_checks.bash`、
`/tmp/task6_quality_thresholds.bash`（再次运行应换输出名）。

## 录制容量：现有实现无需重复改压缩

现有 `tools/task6_ops/evidence_recorder.py` 已订阅 `/image_raw/compressed`，
支持 `--image-hz 2 --max-mib 180`；默认image-hz=0/max-mib=0，统一ops入口才显式限额。
仅丢图像超频帧，scan/odom/tf等不降采样。health有每topic序列化data_bytes及计数、
bag_file_bytes，取两次health差即可估算实际增长，禁止live SQL COUNT。
容量检查每秒一次，因此180MiB是触发停录门槛，非绝不超出一字节的配额。

已只读测量闭合历史bag：18.559531262s、2,981,888B，约160.7kB/s；
只含scan双路、filteredodom、tf/static，还没有图像/底盘odom/imu/rosout。
两路scan各186条、每条5820B。若保持该基线，180MiB仅约1175s，
1800s至少约276MiB；不能承诺“2fps即可连续30分钟不超180MiB”。

实用预算：启动后10–30s，以health的总data_bytes和bag_file_bytes增长取保守速率，
加额外余量，预计覆盖动作时长才启动路线。只在准备/动作/结束必要窗口录制；
UI等待期间另停闭包，下一动作前新bag健康门槛重验。
图像2fps仍过大时可经明确配置降低到1fps，不偷偷降低scan/odom/tf。
录制停止/配额不足应停止对应验收动作，不能继续运行后补录。

UI显示1638/2048MB意味着约410MB余量，180MiB约188.7MB；
两轮满额约377.5MB将几乎耗尽余量，且日志/构建也占空间。
宿主disk_usage剩57GB不能代表Rosject配额。没有删除任何原证据。
先保留当前包；如需长期归档，复制闭合bag+metadata+points+audit+版本/参数+hash清单到
本地指定证据目录，验证SHA及只读回放/消息计数后记录归档位置。
复制归档不会释放云配额；原云证据清理必须另有明确决策，本工具不会自动删。

### 2026-10-22 full03 实际录制预算补充

root已核验该录制 `closed_metadata=true`。实际elapsed=22.396963s，
bag_file_bytes=5,308,416，约237.015kB/s。各topic记录为：
双scan各218条/1,268,760B；tf709/157,620B；filteredodom436/315,664B；
baseodom541/391,684B；cmd2380/123,760B；rosout23/7,980B；
compressed39/1,032,360B；imu541/175,284B。
这些是本轮真实全套录制证据，更新前述5topic历史估计，但不能当未来恒定速率。

保留图像平均26,470.8B/张，本轮保留1.741Hz（2Hz为上限，非保证频率）。
按本轮bag速率，180MiB约796.3s（13.27min），7分钟约94.94MiB（99.55MB）。
7分钟动作还须为启动、停稳、结束写盘和增长波动预留余量；
低速返程不能仅据这一个短样本承诺13分钟内完成。
每次run仍从health差值重新预测，长返程若需分bag，必须保持同一控制会话/claim，
在已经验证的停点闭旧包、开新包并通过健康门槛后才动；不可在运动中留证据空窗，
也不可通过重启返程runner绕过一次性claim。当前实现尚没有这个换包功能。


闭包门禁：显式 --confirm-closed、metadata.yaml 和无 SQLite WAL；同 session 存在 result.json 时必须 closed_metadata 为布尔 true。无 result 的标准包仅依赖显式关闭确认，报告 recorder_closed_metadata_verified=false。归档 supervisor.pid 不用于本机进程探测，避免跨主机 PID 复用假拒绝；该门禁不认证来源身份。
