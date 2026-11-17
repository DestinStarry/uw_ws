# 启动与共享配置

robot.launch.py 和 simulation.launch.py 使用同一 config 参数，默认 config/robot.yaml。
控制增益、死区、稳姿/回中策略、静力矩前馈、名义推力几何共用。
保留的 moment_sign 后端覆盖仅适配已存在的实物/模型坐标约定，迁移时须逐轴核验。

simulation.launch.py 额外读取 plant 参数，默认 config/plant.yaml，用于执行器、传感器和水动力。
其中时间常数、延迟、死区、噪声等是估计初值，不是测量结果。
支持 headless、tcp、tcp_port、publish_truth；最后一项默认 false，真值不供控制器使用。
robot.launch.py 默认 dry_run=true、imu=false、tcp=true、camera=false，未打开真实串口。

校准来源、修改影响和迁移步骤见 docs/实物迁移与标定.md。
