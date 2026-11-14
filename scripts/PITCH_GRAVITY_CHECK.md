# 上 pitch 重力前馈 A/B 验证

本测试新增 `ptest_mode=2` 固定目标模式，需先烧录本次固件一次。后续角度和等待时间通过脚本/var 改，无需烧录。
模式 2 沿用原 pitch PID、CAN 和 IMU，正常叠加重力前馈；不使用起动力矩扫描的偏置。
其他测试模式仍屏蔽重力前馈，避免重复补偿。

遥控开启，双 pitch 正常使能，下 pitch 能保持原位置后，关闭串口助手，运行：

```powershell
python scripts/pitch_gravity_check.py --port COM8 --angles -40 -30 -25
```

每个角度依次测试 0%、50%、100%、再次 0%。目标以 2°/s 平滑切换，目标到位后等待 8 秒，采集 6 秒。
默认测试约 3 分钟。若 PID 积分收敛较慢，可使用 `--settle 15 --measure 10`。
脚本不修改 PID 增益，关闭 DBG 流式输出，持续发送 PC 心跳，并检查 MCU 实际目标与补偿强度再采点。
与起动力矩测试相同，角度范围、跟踪误差、输入和电机状态保护继续生效。
测试结束或 Ctrl+C 后请求停止，MCU 保持当前姿态；脚本恢复测试开始前的重力补偿强度。
这不是失能命令。遥控关闭仍按原 Retry 流程失能。

输出：

- `pitch_gravity_check.csv`：每个阶段采样区间的 MCU 时间、实际目标、IMU 角、PID、前馈、总指令、反馈估算力矩、强度。
- `pitch_gravity_check_summary.csv`：角度误差均值/RMS、标准差、峰峰值、漂移、PID/前馈/总力矩均值。

日志使用整数 mdeg/mN*m，分析结果换成 deg/N*m。
两次 0% 基线不同，或基线仍明显运动/抖动时，不作优劣结论。
其他阶段只有在 PID 绝对均值减小、误差和抖动不明显恶化且未明显漂移时，才报告满足改善判据。
阈值是工程对照标准，不是模型正确性的证明；查看表格后再决定是否保留完整补偿。
补偿正确时总保持力矩通常不减小，而是 PID 输出由重力前馈分担。

重新分析：

```powershell
python scripts/pitch_gravity_check.py --file pitch_gravity_check.csv
```

手动配置接口（仅供调试）：

```text
var set ptest_mode 2
var set ptest_hold -30
var set pitch_up_gscale 0.5
```

测试启停和心跳仍使用 `ptest_on` / `ptest_feed`。推荐用脚本自动处理。
