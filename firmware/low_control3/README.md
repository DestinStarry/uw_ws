# low_control3：以实测 low_control2 为基准的执行层

> 摄像头更新：第七路现为±90°摄像头目标，初始0°；OFF/超时保持角度，夹爪语义已废止。见[摄像头控制](docs/摄像头舵机控制.md)。旧对比描述的是本次摄像头更新前的基准。

2026-11-26 更新：移入当前 low_control2 实测成功的 CH9434 SPI 时序和启动配置验证，保留原推进器方向、PWM范围、舵机和夹爪映射。low_control2 原工程不改动。旧 low3 的详细差异、此次改动及证据见 [low2与low3对比及重构](docs/low2与low3对比及重构.md)。

STM32 负责四路推进器 PWM、三路舵机 PWM、电池电压及四路电调转速。IMU、MS5837 深度计、相机、Orin CPU 温度均由 Orin 负责；STM32 不再轮询旧板载 IMU/深度板，不上传占位深度和温度。

## 当前行为

- 上电先输出1500μs中位，再启动PWM。四个电机 CTRL 方向反相与low2一致，PWM范围1100～1900μs；舵机原通道/限位不变。舵机角度到PWM改为low2的截断算法，避免与原版有1μs量化差异。
- 完整合法CTRL直接执行；OFF停止；连续超过200ms无有效CTRL则中位。下一个有效CTRL可恢复，无SYNC、MCU心跳、回中握手或重新解锁门槛；RPM故障不禁止控制。
- UART中断只缓存字节，主循环按行解析，支持LF、CRLF、半帧和粘包。OFF优先；正常完整OFF后首条新命令不再被误丢。受损残帧先恢复分帧。
- 恢复low2数字命令：`1 1600`～`4 1600`控制电机，`5 1600`～`7 1600`控制舵机，`8 1`蜂鸣。数字PWM不做方向反相、不刷新CTRL超时；独立手动测试先OFF，输出保持到新命令/OFF。不要与Orin持续CTRL/OFF混用。
- SPI2模式0、/128（42MHz APB1时328.125kHz）、DWT至少10μs间隔、2ms单字节超时。等待上电后先做SCR五组测试，再配置UART并读回，启动最多3次。运行中只继续SCR检查与收数，不反复执行阻塞初始化。
- 四路FIFO每10ms调度都轮询，每路最多128字节；不再把积压数据整块丢掉。保留滑动CRC错位恢复、连续两帧确认及500ms反馈超时；全零噪声无效，真实零转速有效。
- 正常遥测10Hz：`VOLT:23.80 CH0:0 CH1:2314 CH2:-- CH3:--`。无效/过期为`--`。每5秒追加一行精简RDBG，详细DIAG默认关闭。
- 保留原low3的独立SysTick命令超时和IWDG；不添加与遥测绑定的控制状态机。

## 编译和烧录

```powershell
.\scriptsuild.ps1 -Configuration Debug
.\scriptsuild.ps1 -Configuration Release
```

脚本自动寻找STM32Cube GNU工具链并运行主机测试，只编译、不烧录。VS Code F5保持CMSIS-DAP配置和构建前置任务，使用`build/Debug/low_control3.elf`。Release产物为`build/Release/low_control3.elf`、`.hex`、`.bin`。

本次Debug、Release编译通过，7组主机测试通过；low3尚未上板验证。测试包含low2实测2314 RPM原始帧、四路积压接收、初始化失败及通道/PWM公式对照。主机测试不代表实际波形、PWM停机时延或IWDG复位已经测量。

配置集中在`userCode/core/Inc/BoardConfig.h`。电机方向和极对数仍为原值（极对数7需核对实物）；`kLegacyPwmEnabled`可关闭数字命令，`kRpmSummaryEnabled`可关闭5秒摘要，`kDiagnosticsEnabled`可开启旧详细DIAG。

协议见[协议与安全行为](docs/协议与安全行为.md)，测试顺序见[上机验收](docs/上机验收.md)，构建记录见[验证记录](docs/验证记录.md)。`low_control.ioc`仅为旧板外设参考，不是本重构完整生成源；不要用它覆盖手工维护的SPI、启动和通信代码。
