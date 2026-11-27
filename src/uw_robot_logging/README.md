# uw_robot_logging

独立下水测试记录包，不发布控制或执行器指令。默认 50Hz CSV 快照，后台有界队列写入，配套事件、配置和运行参数；磁盘故障不作为控制组退出条件。

- 节点：flight_logger_node
- 状态：/logging/status（std_msgs/String JSON）
- 服务：/logging/start、/logging/stop（std_srvs/Trigger）
- 分析：ros2 run uw_robot_logging log_summary <会话目录>
- 完整使用与字段说明：[下水日志与分析](../../docs/下水日志与分析.md)

`schema.py`/`adapters.py` 负责快照与字段单位；`writer.py` 不依赖 ROS；`node.py` 订阅消息与异步获取参数；`summarize.py` 流式读取日志。测试在 test/。

当前schema_version=2，CSV为230列，新增SYNC握手编号、MCU故障代数与RXSTAT接收分类计数；SAFETY控制状态约10Hz更新，DIAG仍1Hz。旧日志仍可用摘要工具读取。


摄像头更新：`OFF:<第七路归一化目标>`仍记录为停机OFF；`ctrl_gripper`为保留的历史列名，现在存第七路摄像头归一化目标，乘90得到度数。该列不是实际角度反馈，`gripper_closed`消息字段也不再控制摄像头。原始hardware_tx保留完整帧，不改变CSV列数。
