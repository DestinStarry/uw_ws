# OpenCV双目相机

相机功能位于独立包 `uw_robot_camera`。控制、分配、电机方向和串口协议保持现有实现。

## 模块分工

| 文件 | 职责 |
|---|---|
| `capture.py` | 唯一打开USB相机的模块；采集与JPEG编码分开线程，保存最新帧，不累积队列 |
| `stream.py` | HTTP视频、图片、健康接口，不接触机器人控制 |
| `camera_node.py` | ROS参数、左右图像发布、启动和退出清理 |
| `robot.yaml` | 分辨率、采集频率、JPEG质量、端口、左右交换等配置 |

采集线程读取2560×720 BGR整帧，按同一采集序号拆为左右1280×720。
编码线程只处理最新整帧，较慢时跳过旧帧，避免形成不断增长的视频延迟。
UI继续使用HTTP，无需让Windows安装ROS2。ROS图像只有订阅者存在才序列化发布。
原始图像发布与JPEG编码不经过控制节点；摄像头失败不会使机器人控制节点退出。

## 首次准备

在实际接相机的Linux主机上关闭旧服务，不要同时运行旧Python main.py占用相机：

```bash
sudo systemctl disable --now mjpg-streamer.service
```

此处关闭自启由用户在香橙派执行；同步WSL源码不会修改香橙派服务。

Ubuntu ROS2环境依赖包括OpenCV、NumPy和sensor_msgs，可通过rosdep安装package.xml依赖。
ROS2节点必须在已安装对应ROS2发行版的主机运行；原香橙派Ubuntu20.04并不因此自动具备Humble运行环境。

## 构建和一键启动

```bash
cd ~/uw_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uw_robot_camera uw_robot_bringup
source install/setup.bash
# 测试控制节点与相机，串口保持dry-run，IMU默认关闭：
ros2 launch uw_robot_bringup robot.launch.py camera:=true
# 完整实物启动：会打开串口，必须在硬件和ROS环境准备好后使用
ros2 launch uw_robot_bringup robot.launch.py camera:=true imu:=true dry_run:=false
```

Ctrl+C统一退出。普通`robot.launch.py`默认`camera:=false`，无相机也能做控制测试。
单独测试相机（不启动控制、串口和IMU）：

```bash
ros2 launch uw_robot_bringup camera.launch.py
# WSL没有USB摄像头时，仅用于验证接口的合成测试图：
ros2 launch uw_robot_bringup camera.launch.py synthetic:=true http_port:=18080
```

合成图明确在启动日志与`/health`中标识，不代表真实相机测试结果。
参数不支持运行中热修改，修改YAML后重启节点。

## 接口和左右顺序

- `http://主机IP:8080/?action=stream` 或 `/stereo.mjpg`：双目拼接流，保留相机原始左右排列。
- `/snapshot.jpg`：相机原始拼接JPEG；`/left.jpg`、`/right.jpg`：按swap_eyes标定后的左右图。
- `/health`：帧年龄、采集与编码FPS、分辨率及异常。旧帧不会继续被当作新采样；无有效图片时快照返回503。
- `camera/left/image_raw`、`camera/right/image_raw`：sensor_msgs/Image，BGR8、传感器QoS。
- 左右ROS图像来自同一Frame并带相同时间戳；这是主机收到帧的时间，不是硬件曝光时间。
- 默认`swap_eyes: true`与此前遮挡测试一致。拼接流不再交换，Windows UI继续保留`VIDEO_SWAP_EYES = True`，避免双重交换。
- 未提供CameraInfo或标定数据，不是已校正深度图；后续识别/测距节点订阅左右图像，完成标定、校正和匹配。

## 性能调节

默认保持2560×720、请求30FPS，JPEG质量由90降到80。实际FPS以`/health`及UI为准。
香橙派此前约8FPS，分离采集/编码减少排队，不保证编码速度能达到30FPS。
`publish_rate_hz: 15.0`只限制ROS原始图像发布，不代表相机只采15FPS。
不需要ROS图像时不要运行图像订阅器，避免额外序列化开销。
若CPU仍不足，可改width=1280、height=480验证；每目640×480是4:3，当前UI的16:9框会保留侧边空白。
WSL使用真实USB相机需要设备已映射到WSL，Windows上的摄像头不会自动成为`/dev/video0`。

## 本次验证记录

- ROS2 Humble环境成功构建 `uw_robot_camera` 和 `uw_robot_bringup`。
- `colcon test --packages-select uw_robot_camera`：4项通过。
- 验证左右顺序、HTTP快照/流、旧帧503、线程清理。
- 验证每目1280×720的ROS Image序列化/反序列化、BGR8步长、左右相同时间戳。
- 合成图相机launch：UI地址兼容、正常退出通过；报告位于 `validation/camera_launch.json`。
- 原控制launch进程生命周期回归通过；报告位于 `validation/launch.json`。
- WSL跨进程图像订阅出现DDS发现/缓冲错误，尚未验证通过；消息序列化通过不能替代跨进程传输验收。没有为规避该问题修改全局DDS或机器人控制配置。
- 未连接真实USB相机，尚未测量香橙派实际FPS、CPU占用或水下效果。

复查命令：

```bash
cd ~/uw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select uw_robot_camera --event-handlers console_direct+
colcon test-result --test-result-base build/uw_robot_camera --verbose
python3 tools/camera_launch_smoke.py
```
