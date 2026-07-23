#!/bin/bash

# 确保ROS环境已加载
source /opt/ros/noetic/setup.bash  # 根据你的ROS版本修改路径

# ==================== 就绪检测函数 ====================
# 均带超时保护:就绪则立即继续,未就绪则在超时后返回非0(由主流程决定是否终止)

# 等待 roscore 可用
wait_roscore() {
    echo "[WAIT] 等待 roscore..."
    local t=0
    until rostopic list >/dev/null 2>&1; do
        sleep 1; t=$((t+1))
        [ $t -ge 60 ] && { echo "[ERROR] roscore 60s 未就绪"; return 1; }
    done
    echo "[OK] roscore 就绪 (用时 ${t}s)"
}

# 等待 MAVROS 真正连接飞控 (/mavros/state 的 connected==True), 并打印 mode/armed
wait_mavros_connected() {
    echo "[WAIT] 等待 MAVROS 连接飞控..."
    local t=0 state mode armed
    while true; do
        # timeout 防止 rostopic echo 在无消息时永久阻塞
        state=$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null)
        if echo "$state" | grep -q "connected: True"; then
            mode=$(echo "$state"  | grep "^mode:"  | head -1 | sed "s/^mode: *//; s/'//g")
            armed=$(echo "$state" | grep "^armed:" | head -1 | sed "s/^armed: *//")
            echo "[OK] MAVROS 已连接飞控 (用时 ${t}s) | mode=${mode:-未知} armed=${armed:-未知}"
            return 0
        fi
        sleep 1; t=$((t+1))
        [ $t -ge 120 ] && { echo "[ERROR] MAVROS 120s 未连接飞控, 请检查 /dev/ttyACM0 权限、波特率 57600、飞控供电与固件"; return 1; }
    done
}

# 等待指定 ROS 节点出现
wait_node() {
    local node="$1" t=0
    echo "[WAIT] 等待节点 $node ..."
    until rosnode list 2>/dev/null | grep -qx "$node"; do
        sleep 1; t=$((t+1))
        [ $t -ge 60 ] && { echo "[ERROR] 节点 $node 60s 未出现"; return 1; }
    done
    echo "[OK] 节点 $node 就绪 (用时 ${t}s)"
}

# 等待指定 nodelet 加载到 manager
wait_nodelet() {
    local mgr="$1" name="$2" t=0
    echo "[WAIT] 等待 nodelet $name 加载到 $mgr ..."
    while ! rosservice call "$mgr/list" 2>/dev/null | grep -q "$name"; do
        sleep 1; t=$((t+1))
        [ $t -ge 60 ] && { echo "[ERROR] nodelet $name 60s 未加载"; return 1; }
    done
    echo "[OK] nodelet $name 已加载 (用时 ${t}s)"
}

# 关键步骤失败时终止整个启动流程(已启动的终端需手动关闭)
fatal() {
    echo "[FATAL] $1"
    echo "[FATAL] 启动流程已终止。已开启的终端(roscore/mavros 等)请手动 Ctrl+C 关闭后重试。"
    exit 1
}

# ==================== 启动流程 ====================

# 1. USB 串口权限
gnome-terminal --tab --title="USB" -- bash -c "sudo chmod 777 /dev/ttyACM0; bash"
sleep 3

# 2. roscore (就绪检测替代固定 sleep 5)
gnome-terminal --tab --title="roscore" -- bash -c "roscore; bash"
wait_roscore || fatal "roscore 未就绪"

# 3. 通过串口连接飞控 (等 /mavros/state connected, 替代固定 sleep 25)
gnome-terminal --tab --title="mavros" -- bash -c "roslaunch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:57600; bash"
wait_mavros_connected || fatal "MAVROS 未连接飞控, 后续 ctrl_node 无法获取 odom, 终止"

# 启动publish_car_odom
# gnome-terminal --tab --title="publish_car_odom" -- bash -c "rosrun simulation_utils cmdvel2rviz_keyboard.py; bash"
# gnome-terminal --tab --title="publish_car_odom" -- bash -c "cd ~/HAM_Flight/sh_utils; ./RTK_pubcarodom.sh; bash"
# (HAM_Flight 不需要车里程计, 已禁用)

# 4. 启动 add_bias: 创建 /drone0/manager + sim_odom(轨迹源) —— 须先于 odomRemap/ctrl_node, 提供 manager
gnome-terminal --tab --title="add_bias(sim_odom)" -- bash -c "cd ~/HAM_Flight/sh_utils; ./RTK_traj.sh; bash"
wait_node /drone0/manager || fatal "nodelet manager /drone0/manager 未启动, 后续 nodelet 无处加载"

# 5. 启动 odom_remap: 加载 odomRemap 到 /drone0/manager (就绪检测替代固定 sleep 25)
gnome-terminal --tab --title="odom_remap" -- bash -c "cd ~/HAM_Flight/sh_utils; ./RTK_remap.sh; bash"
wait_nodelet /drone0/manager odomRemap || fatal "odomRemap 未加载, 无法提供 /odom/remap"

# 6. 启动 controller: 加载 ctrl_node 到 /drone0/manager (就绪检测替代固定 sleep 25)
gnome-terminal --tab --title="controller" -- bash -c "cd ~/HAM_Flight/sh_utils; ./RTK_control.sh; bash"
wait_nodelet /drone0/manager ctrl_node || fatal "ctrl_node 未加载到 /drone0/manager, 无法控制"

# 7. 监控 odom_remap 输出
gnome-terminal --tab --title="rostopic" -- bash -c "rostopic echo /odom/remap; bash"

echo "[DONE] RTK 启动流程完成。若某步报 [FATAL], 请按提示排查后再起飞。"
