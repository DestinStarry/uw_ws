# low_control3：STM32F407 执行层

从 `E:/uw-robot/stm32/low_control2` 单独复制并重构；原工程不改动。目标架构是：电脑负责操作，Orin 负责控制/分配、IMU、I2C 深度计与相机，STM32 负责最终 PWM、电压与四路电调转速采集。

本次已启用[精简遥测](docs/精简遥测.md)：电压和四路转速 10Hz，诊断独立 1Hz；正常发送不再产生额外空行。

## 主要改进

- UART6 中断只收字节、组装有界行；主循环处理完整命令。支持半帧、粘包和 CRLF；过长、非文本、过慢帧及队列溢出触发停机和重新同步。
- 严格校验 7 个十进制有限数及 `[-1,1]` 范围，拒绝 NaN/Inf、尾随文本和多余参数。删除旧 `<id> <PWM>` 串口入口，避免绕过保护。
- 上电、OFF、超时或接收故障后，先接收一帧四路推力均为零的 CTRL 才允许运动；后两路转向舵机和夹爪不要求为零。同一批已排队数据中的 OFF 优先，不能被同批后续 CTRL 覆盖。
- 200ms 命令超时由主循环与独立 SysTick 路径检查；SysTick 可在主循环阻塞时直接写中位。只按有效帧的接收时间续期，BUZZ/无效数据不续期。额外启用独立 IWDG（32kHz LSI 下名义约 2 秒），仅主循环喂狗。
- PWM 启动前写入 1500µs，停机立即更新 CCR；保留原四路方向反相表和输出范围。50Hz PWM 存在一个周期以内的寄存器生效延迟，实际停机时延必须实测。
- CH9434 转速由固定切帧改为滑动 CRC8 重同步；连续两帧 CRC 正确后才有效，损坏帧立即失效，500ms 无新有效帧失效；不再把默认 0 或断线旧值当反馈。
- CH9434 单次 SPI 超时由 1000ms 降为 1ms，错误后停止本轮访问、失效反馈并定期重初始化。每轮读一个通道，轮询预算有界；不更改原芯片配置方式。
- 电压检查每次 ADC 操作状态、加长采样时间，只有成功采样才更新有效值。不再采集/上报 STM32 温度。
- UART6 固定 10Hz 上报电压、CH0~CH3，诊断另以 1Hz 上报，不依赖旧深度数据；发送失败释放 busy，丢回调可恢复，帧截断不会被当完整遥测发送。

## 目录

```text
Core/                       CubeMX 外设、时钟、异常和 SysTick 入口
Drivers/                    STM32 HAL/CMSIS
userCode/core/              无 HAL 的命令、分帧、超时状态、RPM 解码和遥测格式
userCode/control/           已校验执行量到电机/舵机的映射
userCode/devices/           UART 接入、安全、遥测、CH9434、ADC 与 PWM
reference/legacy/           不编译的旧深度、板载 IMU 和算法代码
scripts/build.ps1          编译固件和本机测试，不烧录
tests/                     协议测试与假 HAL 故障注入
docs/                      协议、联调及验证记录
```

`Core/Src/main.c` 的 main 作为 CubeMX 参考入口重命名，真正入口在 `userCode/devices/Src/usermain.cpp`。原 `low_control.ioc` 保留作旧外设参考，未完全描述手工增加的 SPI2、安全与应用逻辑；不要直接重新生成覆盖当前代码，需先迁移这些配置和 USER CODE。

## 编译

Windows PowerShell，在已安装 STM32 GNU 工具链、CMake、Ninja 和本机 g++ 的环境：

```powershell
cd E:\uw-robot\uw_ws\firmware\low_control3
.\scripts\build.ps1 -Configuration Release
```

脚本优先使用 PATH 中的 ARM 编译器，也可发现本机 STM32Cube 工具链；可通过 `ARM_GNU_TOOLCHAIN` 指向编译器 bin。若只有 ARM 编译器，可用 `-SkipTests` 只编译固件。

也可直接使用标准 CMake：

```powershell
cmake --preset Release
cmake --build --preset Release --parallel 2
```

产物：`build/Release/low_control3.elf`、`.hex`、`.bin`。Debug 预设同理。默认 IWDG 不随调试断点冻结，长时间停在断点可能复位；仅在断开推进器动力的台架上，才考虑通过 CMake `-DLOW_CONTROL_ENABLE_IWDG=OFF` 临时禁用。正式测试保持默认开启。

本机测试独立于 ARM 构建：

```powershell
cmake -S tests -B build/host -G Ninja
cmake --build build/host --parallel 2
ctest --test-dir build/host --output-on-failure
```

## 与当前 Orin 配合

UART6：PG14 TX、PG9 RX，115200、8N1；TX/RX 交叉并共地。引脚、PWM 与 ADC 分压均沿用原板，不按通用 F407 开发板替换。

保留 `CTRL:f1,f2,f3,f4,s1,s2,gripper\n` 和 `OFF\n`；现有 Orin `hardware_node` 可继续使用，电压/转速字段保持 `VOLT`、`CH0~CH3`。无效值发 `--`，当前 Orin 严格解析器会使缓存失效。Orin CPU 温度由其自己的监测节点上传，STM32 不重复发 TEMP。

上报的 MCU_SEQ/MCU_MS/VALID/AGE_MS 是额外诊断，当前 Orin 只提取 VOLT 与 CH 数值，未把所有 MCU 诊断转到电脑界面。详见 [协议](docs/协议与安全行为.md)。

第一次联调先断开推进器动力，验证输出中位、OFF、断链与故障后回中恢复；再依厂家要求在允许的环境逐路低幅测试。**尚未烧录或完成实物验证，不能把离线测试当作带动力验收。** 具体顺序见 [上机验收](docs/上机验收.md)。

## 参数与仍需确认的事项

`userCode/core/Inc/BoardConfig.h` 集中定义电机方向、四路极对数、上报/采样周期及 SPI 预算。极对数暂保留原值 7，须按实际电机确认。现有 ESC 遥测没有可靠方向位，上传的是原协议 ERPM 换算后的转速大小，不推断实际正反方向。

CRC8 只有 8 位且电调帧没有固定头，连续两帧匹配能降低误锁概率，不能作为绝对正确性保证。CH9434 芯片配置和 SPI 波形仍需实物检查。电压仍使用原板分压比及名义 VREFINT=1.2V，需要万用表对照标定，未新增低电压自动切断阈值。

CTRL 下行保持旧文本格式，尚无校验和/序号/独立解锁位；下一版如增加，应同步修改 Orin，而不能仅单边换协议。当前“零推力恢复”不是独立硬件解锁，外部急停/动力切断仍需保留。

独立看门狗可复位卡死程序，但复位期间引脚状态、电调失信号行为和最终停机时间必须实测；固件不能代替物理断电急停。

## Git 仓库目录

此目录由外部 low_control3 源码同步收录。VS Code 应单独打开本目录后使用其 CMake/调试配置，避免 ${workspaceFolder} 指向 ROS 仓库根目录。烧录器配置沿用本机已修改的设置；OpenOCD/GDB 本机路径在其它电脑上需按实际安装修改。COLCON_IGNORE 位于 firmware 上级，ROS 构建不会尝试编译本工程。
