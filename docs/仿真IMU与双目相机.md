# 仿真 IMU 与双目相机

## 安装坐标

以艇体 base/base_link 为参考，X 前、Y 左、Z 上，单位米。

| 传感器 | 位置 | 说明 |
|---|---|---|
| IMU | (0.06, 0, 0.04) | 用户指定近似位置；轴向与艇体一致 |
| 左目光心 | (0.285, 0.03, 0) | 前端半圆罩附近，视线沿 +X |
| 右目光心 | (0.285, -0.03, 0) | 与左目平行，基线 60mm |

IMU 位于艇体前方6cm、上方4cm。IMU 改动只移动测量点，不改变艇体质量、重心或浮心。
偏离重心的加速度计会自然测到角运动带来的切向/向心加速度；姿态与角速度仍测量艇体运动。
IMU 的 ROS frame_id 为 imu_frame，发布 base_link -> imu_frame 静态变换。

双目基线为用户确认的60mm；安装中点和60度垂直视场角是近似值，需要实物光心测量/标定后更新。
相机作为 MuJoCo 固定视点，不增加活动关节、质量或碰撞。
光学坐标为X右、Y下、Z前，已发布与机体坐标系之间的静态变换。

## 启动和预览

```bash
source /opt/ros/humble/setup.bash
source ~/uw_ws/install/setup.bash
ros2 launch uw_robot_bringup simulation.launch.py headless:=false
```

默认开启双目，每目640×360，目标频率10Hz；两目拼接为1280×360。
实际帧率取决于渲染设备，HTTP /health 返回实测capture_fps与图像年龄。
默认场景只有地面与背景，前方没有目标时图像主要显示地平线和地面，不是相机故障。

同机预览地址：

- http://127.0.0.1:8080/?action=stream （兼容原 UI）
- http://127.0.0.1:8080/stereo.mjpg
- http://127.0.0.1:8080/stereo.jpg
- http://127.0.0.1:8080/left.jpg
- http://127.0.0.1:8080/right.jpg
- http://127.0.0.1:8080/health

Windows UI 的视频地址指向该仿真流，而非原机载电脑IP。拼接顺序为物理左目在左、物理右目在右，仿真输出不额外交换左右目。

```bash
# 暂时不需要图像时关闭，物理仿真和IMU照常运行
ros2 launch uw_robot_bringup simulation.launch.py camera:=false
# 8080已被其他服务占用时
ros2 launch uw_robot_bringup simulation.launch.py camera_port:=8081
```

## ROS 接口

- imu/data：sensor_msgs/Imu，frame_id=imu_frame。
- camera/left/image_raw、camera/right/image_raw：sensor_msgs/Image，BGR8。
- camera/left/camera_info、camera/right/camera_info：sensor_msgs/CameraInfo。
- tf_static：IMU和左右光学坐标系相对于base_link的安装变换。

图像与内参仅在有ROS订阅者时发布，HTTP预览持续可用；ROS图像使用SensorData QoS。
两目从同一物理状态快照渲染，共用时间戳；右目投影矩阵Tx=-fx×baseline。
内参对应理想无畸变针孔模型，不模拟实物罩体折射、浑浊、曝光和畸变。

## 参数和实现

安装位置：uw_robot_mujoco/assets/auv-v3.xml 的 imu_site 和 stereo_left/stereo_right。
图像参数：uw_robot_bringup/config/plant.yaml 中 camera_* 参数。
运行时camera_position_m与camera_baseline_m会覆盖XML相机位置，TF与内参随之更新。
IMU位置以XML为准，不要只改TF而不改传感器实际测量点。

渲染线程拥有独立MjData及OpenGL上下文，物理循环只传最新qpos/qvel快照，不排队历史视频。
物理模型不向控制器新增位置或速度真值反馈；传感器移动不改变原闭环目标或增益。
为降低WSL图形缓冲开销，阴影图使用1024、多重采样关闭。
headless只关闭MuJoCo交互窗口；需要离屏图像时仍需要可用OpenGL后端。
默认使用已有WSLg/GLFW；仅在支持EGL的环境显式设置MUJOCO_GL=egl。
如果渲染失败，记录错误并保留IMU/物理仿真；检查图形驱动或暂用camera:=false。

验证包含：IMU安装/偏心加速度、双目基线/投影、实际目标视差及渲染不修改物理状态。
