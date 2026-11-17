# 公共接口包

此包只定义 ROS 消息和纯 C++ 数据类型，不启动节点。

| 消息 | 含义 |
| --- | --- |
| `ControlCommand` | 机体系六轴速度指令与四个离散命令 |
| `WrenchCommand` | 机体系六维目标及使能、会话、epoch |
| `ThrusterCommand` | 四推进器推力、两组倾转角和夹爪目标 |
| `DepthState` | 深度、变化率、有效性及数据年龄 |
| `RobotStatus` | 控制模式与控制目标诊断 |
| `AllocationStatus` | 规划状态、降额目标与模型预计合力 |

`include/uw_robot_interfaces/robot_types.hpp` 是算法内部的 Eigen 类型，避免数学代码依赖 ROS 节点。
六维排列 `[Fx,Fy,Fz,Mx,My,Mz]`，单位 N、N·m；角度 rad，三维向量 xyz。
执行器内部排列 `[1上,3下,2右,4左]`，不能直接按编号重新排序。
倾转字段保留既有名称 `theta12_rad/theta34_rad`，实际对应内部 1/3、2/4 组。

`ControlCommand.velocity` 按 `[u,v,w,p,q,r]` 排列：线速度 m/s（x 前、y 右、z 向下），
角速度 rad/s（绕 x、y、z）。手柄轴映射、死区和滤波都在发送端完成，控制节点只跟踪物理量。
`header.stamp` 和数据年龄共同判断新鲜度，状态消息中的 achieved 值是模型推算，不是真实推力测量。
修改 `.msg` 会影响所有依赖包；本次由 `ControlRequest` 改为 `ControlCommand`，字段与语义同步变更。
