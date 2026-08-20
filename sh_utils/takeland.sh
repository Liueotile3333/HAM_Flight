#!/bin/bash
# 参考 takeoff.sh: 发 TakeoffLand(takeoff_land_cmd=2 即 LAND) 到 ctrl_node 私有话题。
# FSM 须处于 HOVER/MISSION 且水平近静止(轨迹结束后悬停在 final pose, 水平速度<0.1);
# 否则 LAND 触发会被拒绝(运动中降落会急刹, 不安全)。
# 降落流程: HOVER/MISSION --> LAND --> 立即请求 PX4 AUTO.LAND（由 PX4 land detector 自动 disarm）。
# 注意: takeoff_land_cmd 为 1(TAKEOFF) 触发起飞, 为 2(LAND) 触发降落, 其余值兼容当起飞。
rostopic pub -1 /drone0/ctrl_node/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
