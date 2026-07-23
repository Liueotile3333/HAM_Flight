# FSM(ctrl_node) 用 private nh 订阅 takeoff_land,节点名为 /drone0/ctrl_node
# 故 topic 实际为 /drone0/ctrl_node/takeoff_land (须带 drone0/ 前缀)
rostopic pub -1  /drone0/ctrl_node/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
