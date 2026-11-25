# uw_robot_logging

独立下水测试记录包，不发布控制或执行器指令。默认 50Hz CSV 快照，后台有界队列写入，配套事件、配置和运行参数；磁盘故障不作为控制组退出条件。

- 节点：flight_logger_node
- 状态：/logging/status（std_msgs/String JSON）
- 服务：/logging/start、/logging/stop（std_srvs/Trigger）
- 分析：ros2 run uw_robot_logging log_summary <会话目录>
- 完整使用与字段说明：[下水日志与分析](../../docs/下水日志与分析.md)

`schema.py`/`adapters.py` 负责快照与字段单位；`writer.py` 不依赖 ROS；`node.py` 订阅消息与异步获取参数；`summarize.py` 流式读取日志。测试在 test/。
