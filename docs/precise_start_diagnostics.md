# Precise-start 诊断字段

本片段只记录已有状态。5 秒采集预算、0.5 秒停稳、15 帧/12 帧一致性、
10 mm 距离全幅、2° 角全幅、最终 5 mm / 0.5°、恢复时限、细调次数及控制输出均不改变。
`wall_valid/stable` 属于独立的墙朝向估计，不能替代精细位置批次验收。

`START_DIAGNOSTIC` 仅在验收通过、细调、细调耗尽、超时或反馈恢复清样本时输出；
不逐帧记录。`cycle` 每次开始一轮精细验收加一；以下计数在每轮开始清零：

- `sample_count`：当前尚未形成完整批次的样本数；`accepted`：本轮累计入批帧数。
  无效朝向以 NaN 入批，是否共同有效仍由原有 `summarize_start` 判断。
- `batches`：本轮完整批次数。大于零时，同一事件另输出
  `START_DIAGNOSTIC_LAST_BATCH`，保留实际最后一批的 retained/stable/中值/全幅。
  retained 小于 12 时，中值/全幅默认值不能解释为测量零误差。
- `reset_events`：停稳阻塞首次出现、原因变化或已有样本被清空的次数；
  `linear_resets/angular_resets/anchor_resets` 可同时增加。
  `last_stop_mask` 的位 1/2/4 分别表示平移速度、转速、停稳锚点位移或角差超限。
  连续相同阻塞只增加 `blocked_ticks`，不重复计为新事件。
- `discarded`：因停稳/反馈恢复清空的样本数，不包含完成批次的正常清空。
  `hold_ticks/no_new_scan_ticks` 分别表示等待停稳时间或没有新时间戳的控制周期数，
  后者也会出现在控制周期快于正常雷达频率时，不能直接等同丢帧。
- `scan_resets/odom_resets`：本轮遇到对应反馈恢复时加一，清空并发布零速后输出；
  恢复后新一轮会重新计数，通过 `cycle` 和恢复事件连接证据。
- `odom_source_age/scan_source_age` 使用节点时钟减消息时间戳，
  `odom_receipt_age/scan_receipt_age` 使用 steady clock 减最后接受反馈的接收时刻。
  这是日志事件时的年龄，不是整段历史最大值。scan source 指最后转换的观测，
  scan receipt 指最后有效测距帧；请与 scan_reason 一起读。

最小提取：

```bash
grep -nE 'START_VERIFY|START_DIAGNOSTIC|START_REFINE|SCAN_RECOVER|ODOM_RECOVER|Initial preparation fault' "$RUN_DIR/motion.log"
```

本地验证入口：`test_start_verification` 包含诊断累计和原批次不稳定结果保留；
真实 ROS fixture `test/verify_wall_session.py` 的 `PRECISE_DIAGNOSTIC=stop_velocity`
与 `slow_scan` 分别区分停稳不满足和帧不足。保持失败、零速度、原 5 秒时限和无交接。
`PRECISE_NOISE=unstable` 验证最后批次可追溯，普通及 scan 恢复路径检查成功验收未变。
fixture 仅使用 localhost，`WALL_TEST_DOMAIN` 可选择独立域，默认仍 181。

实现审阅：`verify_precise_start` 增加被动计数后略超 60 行；保留原控制判断与时序顺序，
本诊断片段不为格式阈值重构状态机。计数器及日志函数已独立；C++/Python格式和检查照常执行。
