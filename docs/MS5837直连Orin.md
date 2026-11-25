# MS5837 直连 Orin

## 数据链

MS5837 → Orin I2C → depth_node → depth/state → control_node / tcp_bridge_node。
STM32 继续负责执行器，不再参与默认深度采集；Windows 的 DEPTH / DEPTH_VALID 字段保持兼容。
压力、温度一阶及二阶补偿已用用户提供的 `E:\uw-robot\ms5837\ms5837\ms5837.py`
产生的 8 组数值向量回归覆盖（两种型号，-25/-5/20/40°C）。
用户已在 Orin 实物验证 I2C-7、MS5837-30BA，并得到约 20Hz 的 valid=true 深度数据；水下精度、定深方向及零点仍需现场验收。

## 连接与准备

断电后按实际模块和 Orin 载板说明连接电源、GND、SDA、SCL，核对 I/O 电平及上拉。
不要沿用旧解算板的 UART 接线，也不要按未知载板猜针脚号或总线号。

```bash
sudo apt install python3-smbus i2c-tools
sudo usermod -aG i2c "$USER"
# 重新登录使组权限生效
ls -l /dev/i2c-*
i2cdetect -l
# N 替换成实际接出的总线号；只扫描确认可用于该传感器的总线
sudo i2cdetect -y -r N
```

MS5837 固定 7 位地址 0x76（配置整数 118）；不要填写左移后的 8 位地址。

本机实物曾在 i2cdetect -r 中不显示 0x76，但按手册命令能读校准数据，不能只凭扫描结果认定损坏。停止其它 I2C 访问后可直接确认：

```bash
sudo i2ctransfer -y 7 w1@0x76 0x1e
sleep 0.02
sudo i2ctransfer -y 7 w1@0x76 0xa2
sudo i2ctransfer -y 7 r2@0x76
```

实物曾返回 0x75 0xba；各传感器校准系数不同，不要求字节相同。

不要在采集运行时扫描总线，也不要同时启动另一个 I2C 读程序。

## 配置

编辑 `src/uw_robot_bringup/config/robot.yaml` 的 `depth_node.ros__parameters`。

| 参数 | 默认 | 含义 |
|---|---|---|
| bus / address | 7 / 118 | 实际 I2C 总线及 7 位地址 |
| model | 30ba | 空气中按压力自检；已确认型号后建议固定 02ba 或 30ba |
| oversampling | 4096 | 两种型号均支持 256～8192；非法档位启动即拒绝 |
| fluid_density | 997.0 | kg/m³；按实际水体配置，深度向下为正 |
| surface_pressure_mbar | 0.0 | 0=启动自动标定；正值=使用实测空气零点，跳过自动标定 |
| zero_time_s / zero_attempts | 3.0 / 3 | 每轮标定时长与最大尝试次数 |
| allow_zero_fallback | false | 标定失败默认无效；true 才允许使用估计零点 |
| expected_surface_mbar | 1013.25 | 自动判型参考气压及显式开启的回退气压 |
| rate_hz | 20.0 | 目标采样/发布频率；实际受 I2C 转换时间限制 |
| stale_timeout_s | 0.5 | 样本失效时限 |
| max_failures | 5 | 连续读取失败告警阈值；每次失败都立即置无效 |
| heartbeat_hz | 1.0 | 无效期间状态心跳频率；从有效转无效立即发布一次 |
| frame_id | base_link | 深度消息参考机体系 |

压力范围检查无法识别浅水上电：例如水下数十厘米的压力仍可能落在空气范围。
**自动零点标定必须在空气中静止完成，再下水。** 波动超过 5 mbar 的一轮会被拒绝。
自动判型也只适用于空气中，不能作为可靠的水下型号识别方式。
需水下重启时，先固定真实型号，并填入下水前实测的 `surface_pressure_mbar`。
标定重试耗尽后进入 uncalibrated，保持 valid=false；在空气中重新启动节点进行标定。
断线恢复保留本次已建立的零点与型号，不在水下重新归零；变化率历史清零以免产生尖峰。

## 单独验收深度计

```bash
cd ~/uw_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-up-to uw_robot_bringup
source install/setup.bash
ros2 launch uw_robot_bringup depth.launch.py
# 另一个已加载环境的终端
ros2 topic echo /depth/state
ros2 topic hz /depth/state
ros2 topic info /depth/state --verbose
```

临时指定总线和型号（与上面的 launch 二选一）：

```bash
ros2 run uw_robot_sensors depth_node --ros-args \
  --params-file src/uw_robot_bringup/config/robot.yaml \
  -p bus:=7 -p model:=30ba
```

`7` 和 `30ba` 是当前已验证的实物配置；换载板或传感器后重新确认。
验收顺序：空气中标定时 valid=false → 完成后 valid=true 且深度接近 0 →
增加已知水柱高度，检查正向深度及量级 → 断线后立即无效 → 恢复后深度连续且不重新归零。
无总线、无 SMBus 或 PROM 校验失败会拒绝启动并给出错误；运行期读取失败保留无效心跳并尝试重连。

## 整机启动与旧链路回退

```bash
# 默认 Orin I2C 深度源；仍不打开执行器串口
ros2 launch uw_robot_bringup robot.launch.py depth_source:=i2c
# 无传感器的 WSL 控制链调试
ros2 launch uw_robot_bringup robot.launch.py depth_source:=none tcp:=false
# 旧解算板→STM32 回退（仅真实串口模式才会接收物理数据）
ros2 launch uw_robot_bringup robot.launch.py depth_source:=stm32
# 已完成深度/IMU/串口单独验收后运行实物
ros2 launch uw_robot_bringup robot.launch.py depth_source:=i2c imu:=true dry_run:=false
```

launch 根据 `depth_source=i2c/stm32/none` 强制配置 STM32 的 publish_depth，并选择是否启动
I2C 节点；无需再手动修改两个开关。旧 `depth:=false` 仍可关闭 I2C 节点，不会自动切到旧串口源。
单独启动 hardware_node 时应加载同一个 robot.yaml；不要同时启动多套机器人 launch。
默认 depth_source=i2c 的整机启动需要真实 I2C，dry_run 只关闭执行器串口，并不模拟深度计。

控制策略保持现状：未有有效深度时不能开启定深；已开启定深后深度失效会暂停定深补偿、
清积分，姿态和手动控制继续，深度恢复时重新捕获当前深度。不会因深度恢复自动重新解锁。
操作者应依据 DEPTH_VALID 判断当前深度可用性，失效时不能将 DEPTH_HOLD 标志理解为仍在有效定深。

## 软件回归

```bash
python3 -m pytest src/uw_robot_sensors/test -q
python3 tools/depth_launch_smoke.py
```

包含设备层数值对照、非法参数、型号传递、标定无效、标定失败、断线恢复、真实 ROS 节点消息序列化、
深度源启动互斥。测试用寄存器级 SMBus 替身，不打开真实 I2C 或执行器串口。

资料核对：[MS5837-30BA 原厂数据手册](https://www.mouser.com/datasheet/2/418/5/NG_DS_MS5837-30BA_B1-1130109.pdf)，两种型号均支持 OSR 8192；默认仍用 4096 以控制转换延迟。
