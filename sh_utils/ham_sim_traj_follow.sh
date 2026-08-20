#!/bin/bash
# =============================================================================
# ham_sim_traj_follow.sh  ——  HAM 升力翼仿真【一键启动】
# 把"启动前需手动做的"全部自动化:清理残留仿真 -> 确保 PX4 在路径 ->
# 确保 iris_liftwing 模型/机架存在 -> 启动全链路(Gazebo+PX4+MAVROS+规划器+控制器)。
#
# 用法:
#   bash sh_utils/ham_sim_traj_follow.sh                # 默认升力翼 iris_liftwing
#   bash sh_utils/ham_sim_traj_follow.sh iris_liftwing  # 显式升力翼(带机翼)
#   bash sh_utils/ham_sim_traj_follow.sh iris           # 普通四旋翼(无机翼)
#
# 前提:请在【交互式终端】运行(让 ~/.bashrc 自动 source 好 PX4 与 HAM_Flight);
#       ~/.bashrc 里应已包含 PX4 的 setup_gazebo.bash(一次性配置)。
# =============================================================================
VEHICLE="${1:-iris_liftwing}"                       # 默认升力翼
HAM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"             # HAM_Flight 根目录
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"

echo "==================== HAM 仿真一键启动 ===================="
echo "vehicle = ${VEHICLE}"

# ---------- [1/4] 清理残留仿真(避免旧进程占端口导致新 launch 起不来) ----------
echo "[1/4] 清理残留仿真进程..."
pkill -f gzserver  2>/dev/null; pkill -f gzclient 2>/dev/null
pkill -f " px4 "    2>/dev/null; pkill -f mavros    2>/dev/null
pkill -f rosmaster  2>/dev/null; pkill -f roslaunch 2>/dev/null
sleep 3

# ---------- [2/4] 确保 PX4 在 ROS 路径 ----------
echo "[2/4] 检查 PX4 是否在 ROS 路径..."
source ~/.bashrc 2>/dev/null
if [ -z "$(rospack find mavlink_sitl_gazebo 2>/dev/null)" ]; then
  echo "  ⚠️  rospack 找不到 mavlink_sitl_gazebo —— PX4 未 source。"
  echo "      请确认 ~/.bashrc 里有:"
  echo "        source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default"
  echo "        export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot"
  echo "      然后开一个新终端再跑本脚本。"
  exit 1
fi
echo "  ✓ mavlink_sitl_gazebo 已就绪"

# ---------- [3/4] 确保 iris_liftwing 模型 + PX4 机架存在 ----------
echo "[3/4] 确保 vehicle=${VEHICLE} 的模型/机架存在..."
if [ "$VEHICLE" = "iris_liftwing" ]; then
  SDF="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_liftwing/iris_liftwing.sdf"
  if [ ! -f "$SDF" ]; then
    echo "  ⚠️  iris_liftwing 模型缺失:$SDF"
    echo "      请从 PX4-Autopilot 侧补齐该 SDF(含翼面 + 气动插件 + PX4 机架)后重跑。"
    exit 1
  fi
  echo "  ✓ iris_liftwing 模型/机架就绪"
else
  echo "  (使用内置 ${VEHICLE},跳过)"
fi

# ---------- [4/4] 启动全链路(单窗口多 tab,tab 内自等待上游就绪) ----------
#   所有节点跑在【同一个新 gnome-terminal 窗口】的各 tab 里;当前终端不跑 roslaunch,
#   只等待全链路就绪后交还用户控制。每个 tab 先 wait 自己的上游再 roslaunch,
#   以"等待就绪"取代固定 sleep,保证启动顺序与可靠性。
#
#   ⚠️ gnome-terminal 多 tab 必须用 -e(每个 -e 只吃一个参数,各 tab 独立);
#      切勿用 "-- cmd" 形式 —— gnome-terminal 遇到 -- 会把后续所有 --tab 当作
#      首命令的参数,导致只有第一个 window 生效、其余 tab 全部失效。
COMMON="$HAM/sh_utils/_ros_common.sh"
RATES="$HAM/sh_utils/mavlink_set_rates.sh"
source "$COMMON"                    # 当前终端要用 log / wait_node / wait_topic_messages / die / mk_tab

# mk_tab / cleanup_tabs 取自 _ros_common.sh;退出时清理本进程生成的临时 tab 脚本。
trap cleanup_tabs EXIT

echo "[4/4] 启动全链路(vehicle=${VEHICLE}  ->  Gazebo 模型名 ${VEHICLE}_0)"
echo "      节点开在新终端窗口的各 tab;当前终端等待全链路就绪。"

# 每个 tab:先 wait 上游就绪(失败则 exec bash 保留终端排查),再 roslaunch。
T_BASE=$(mk_tab "roslaunch uav_utils base_sim_single_vehicle_ground_truth.launch vehicle:=${VEHICLE}")
T_RATES=$(mk_tab "wait_node /mavros 90 || exec bash; bash \"$RATES\"")
T_TRAJ=$(mk_tab  "wait_node /mavros 90 || exec bash; roslaunch trajectory_utils traj_gen.launch")
T_CTRL=$(mk_tab  "wait_node /drone0/manager 90 || exec bash; roslaunch ctrl_node run_ctrl.launch")
T_RC=$(mk_tab    "wait_node /manager 90 || exec bash; roslaunch realflight_utils rc_remap_sim.launch")
T_ODOM=$(mk_tab   "wait_node /manager 90 || exec bash; roslaunch realflight_utils odom_remap_sim.launch")

gnome-terminal \
  --window -e "$T_BASE" \
  --tab   -e "$T_RATES" \
  --tab   -e "$T_TRAJ"  \
  --tab   -e "$T_CTRL"  \
  --tab   -e "$T_RC"    \
  --tab   -e "$T_ODOM"

# 当前终端:等待最终链路就绪(/mavros + /odom/remap),再交还用户控制
log WAIT "等待全链路就绪(/mavros + /odom/remap)..."
wait_node            /mavros     120 || die "MAVROS 未在 120s 内就绪"
wait_topic_messages  /odom/remap 3 120 || die "/odom/remap 未就绪(请检查新窗口里 odom_remap / base_sim tab)"

echo ""
echo "==================== 启动命令已发出 ===================="
echo "等 Gazebo 窗口出现 + PX4 报 'SYS_AUTOSTART=10020' 后,在【另一个终端】继续:"
echo "  验证模型:   rostopic echo -n 1 /gazebo/model_states/name      # 应含 ${VEHICLE}_0"
echo "  切 OFFBOARD: bash sh_utils/set_offboard.sh"
echo "  起飞:       bash sh_utils/takeoff.sh"
echo "  启动轨迹:   bash sh_utils/pub_trigger.sh"
echo "  降落:       bash sh_utils/takeland.sh   # 待轨迹结束(悬停 final pose)后再发"
echo "========================================================="
