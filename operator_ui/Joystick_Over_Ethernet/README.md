# Joystick Over Ethernet：ROV 电脑端 UI

电脑经网线连接 Orin：TCP 8888 发送控制、接收状态和传感器数据；HTTP 8080 接收双目 MJPEG 视频。实际控制/分配在 Orin，STM32 负责执行器、电压和转速。

## 启动

沿用已安装 pygame、opencv-python、numpy 的 Python 环境：

```powershell
cd E:\uw-robot\stm32\joy\Joystick_Over_Ethernet
python main.py
```

`config.py` 中 ONBOARD_IP 默认为 192.168.1.10，按实际 Orin 地址修改。新旧 UI 都需要 Orin 控制状态回传 ARMED 才认为控制程序在线；温度/视频不能代替控制心跳。

## 当前显示改动（2026-11-24）

- 明确标为“Orin CPU 温度”，优先使用 CPU_TEMP，仅在不存在该字段时兼容 TEMP 别名。CPU_TEMP 已出现但无效/过期时，不回退显示旧 TEMP。
- 电压、CPU 温度、四路转速遵循 VALID、AGE_MS（如果上位机提供）与每字段本地接收超时；主连接仍在线但某个传感器不再更新时，也会隐藏旧值。
- 缺失、无效、非有限数、超时统一显示 --；真实零转速仍显示 0，不用默认 0 冒充已测得零转速。
- 电压/温度/RPM 接收超时默认 3秒，深度显示 1.5秒、姿态显示 1秒。仅改变显示，不修改 Orin 或 MCU 的控制看门狗。
- 视频使用明确的 `/stereo.mjpg` 左右拼接整帧端点；原相机通常为 2560×720，由 UI 自动缩放。/?action=stream 旧别名仍由 Orin 兼容。
- 保留 VIDEO_SWAP_EYES=True；遮住单个镜头检查左右目标签，不根据截图猜实际安装顺序。HTTP 拼接流在 Orin 端保留原线序，UI 负责这个显示交换。
- 超过 2秒未收到新视频帧，隐藏旧画面并显示等待占位框；视频 EOF 会退出接收，避免无数据循环。当前视频线程不自动重连，恢复相机后可重启 UI。
- 没有修改 command_mapper.py、按键、轴映射、量程或控制发送协议。

## 按键与协议

A：解锁/锁定；B：急停；X：定深切换；Y：开环/闭环切换（由 Orin 判断是否允许）。左摇杆平移、扳机升沉、右摇杆俯仰/偏航，LB/RB 横滚。界面实际模式以 Orin 回传状态为准。

```text
CMD:u,v,w,p,q,r,arm,estop,depth,loop
```

以 LF 结束，约 30Hz。控件读取和指令成形在 command_mapper.py；退出时保留原急停发送流程。不要向此客户端直接输入 STM32 的混合遥测行，混合串口数据由 Orin 桥提取成独立 TCP 字段。

## 文件

| 文件 | 用途 |
|---|---|
| main.py | 界面、手柄读取和双目视频显示 |
| network.py | TCP 连接、控制心跳、每字段接收时间和原子遥测快照 |
| telemetry_display.py | 无图形依赖的有效性、过期判断与显示格式 |
| config.py | IP、视频地址、显示超时和既有控制参数 |
| command_mapper.py | 原有控制映射，本次未改 |

## 离线验证

只测显示规则和本机 socketpair，不连接 Orin：

```powershell
python -m unittest discover -s tests -v
```

生成模拟画面（替换了网络和相机，不发送任何控制）：

```powershell
python tools/preview_ui.py --output validation/ui_preview.png
python tools/preview_ui.py --invalid --output validation/ui_invalid_preview.png
```

原双目分辨率的本地 MJPEG 测试：

```powershell
python tools/check_video.py
```

离线测试通过不代表已连接真实 Orin/相机验收。实际相机应在 Orin 配置为正确的左右拼接分辨率，仅启动一个 camera_node；不要继续用临时单目 1280×720 覆盖参数。


## 摄像头（第七路）

Xbox左下十字键上/下每按一次±10°，长按不重复，范围±90°，初始0°。画面控制栏显示摄像头目标角度。`camera_pan.py`记录按键边沿，`command_mapper.py`将绝对角度附在CMD第11字段。锁定时同样可调整，不随A解锁跳动。需要配套更新Orin及STM32；实际顺转方向可用robot.yaml的hardware_node.camera_direction=−1反向校准。
