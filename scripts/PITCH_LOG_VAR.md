# 单上 pitch：直接用 log / var 交互调参

当前固件从 pitch 控制入口移除了 sweep / breakaway / PSO 测试接管，`CONFIG_TRD_TEST=n`。
旧脚本和旧测试源码保留为历史资料，不适用于当前固件，不再使用 `ptest_*` 命令。
此接口在正常 pitch 线程中执行，继续由遥控整车开关、使能与原掉线检查控制运行。

## 直接交互

由助手直接连接 COM8，逐次发送 shell 命令、采集日志并判断下一项调整，不运行无人审查的参数搜索。
更换本次控制接口需要烧录一次；后续改目标角、参考速度、PID 和补偿强度均用 var，无需烧录。
本次保留源码现有上轴默认值 `12 / 1 / 0.08` 和重力强度 `0.5`，下轴参数不变。

```text
var get pitch_up_kp
var get pitch_up_ki
var get pitch_up_kd
var get pitch_up_gscale

var set pitch_up_target -30
var set pitch_up_rate 2
var set pitch_up_manual 1
log on pitch_up_state 5
```

- `pitch_up_manual=0`：正常遥控目标；`1`：采用 `pitch_up_target`。
- `pitch_up_target`：IMU 目标角，单位度，钳位在 `trd_pitch.hpp` 当前定义的 -55°～-5°。
- `pitch_up_rate`：参考移动速度，度/秒，最大 10；默认 2。
- 启用手动目标或故障恢复时，从当前 IMU 姿态开始建立参考，不直接跳到旧目标。
- 即使 manual=1，也需要遥控整车处于开启、两轴使能等原运行条件满足；该变量不绕过关闭逻辑。

一次只改一项，例如验证积分从 1 增至 2：

```text
var set pitch_up_ki 2
var set pitch_up_reset 1
var set pitch_up_target -32
```

`pitch_up_reset` 在 pitch 控制拍中清除 PID 积分、微分与输出历史，执行后自清；不重新设零、不改变电机零点。
复位有可能改变维持负载的积分力矩，应在小误差、停稳时执行，保留重力补偿与总力矩限幅。
助手比较改动前后在同一角度的静差、运动滞后、过冲、抖动和力矩，再决定继续或回退。
不因一次低评分就宣称参数最优，不把不动当成稳定跟踪。

## 日志格式

`pitch_up_state` 每条 8 个整数：

```text
uptime_ms,actual_mdeg,reference_mdeg,pid_mNm,gravity_mNm,total_mNm,feedback_mNm,task_dt_ms
```

- mdeg / mNm 除以 1000 后为度 / N*m。
- reference 是经限速后的实际参考，可能尚未到达 var 指定的最终目标。
- task_dt_ms 是实际 Task 间隔，供判断调度；本次没有同时改变 PID 的离散计算定义。
- 反馈力矩是最近一帧 CAN 的估算值，不是独立力矩传感器同步采样。
- `log on ... 5` 表示调用分频，不保证固定采样频率；所有时间分析按 MCU 时间戳。

便于人工观察的其他通道：

```text
log on pitch_angle 10        # 实际 IMU 角、实际参考角；两列均为度
log on pitch_up_torque 10    # PID、重力前馈、最终力矩；N*m
log off
```

一次只选中一个 DBG 通道；记录的响应必须注明当时的 PID、重力强度、目标和单位。

## 结束交互

要继续保持当前指定目标，只需 `log off`，manual 模式仍然工作。
要交回遥控：先将遥控输入调整到接近当前姿态，再 `var set pitch_up_manual 0`；遥控目标本身仍沿用原映射。
需要失能则使用原遥控整车关闭操作。串口关闭不会自动失能，也没有旧测试脚本的心跳接管。
