#!/bin/bash

# HAM real-flight one-key startup.
# This script deliberately stops at READY:
# it does not arm the vehicle, switch to OFFBOARD, take off, or start a mission.

FCU_DEVICE="/dev/ttyACM0"
FCU_BAUD="57600"
RTK_STATUS_TOPIC="/mavros/gpsstatus/gps1/raw"
RTK_FIXED_MIN_TYPE=6
MAX_OFFBOARD_LOSS_TIMEOUT="${MAX_OFFBOARD_LOSS_TIMEOUT:-1.0}"

# MAVLink 数据流频率由 mavlink_set_rates.sh 统一下发,实飞频率在下方调用处以
# 环境变量(ID31/ID32/ID331)覆盖,不再在此声明本地常量。

# Prevent two startup scripts from running at the same time.
exec 9>/tmp/ham_rtk_start.lock
if ! flock -n 9; then
    echo "[FATAL] Another RTK.sh is already running."
    exit 1
fi

source /opt/ros/noetic/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 公共 ROS 编排工具(log / wait_node / wait_nodelet / wait_topic_messages /
# wait_roscore / ensure_roscore / wait_mavros_connected / ensure_node_absent /
# read_state / state_value / mk_tab)统一取自 _ros_common.sh,避免两份实现分叉。
source "$SCRIPT_DIR/_ros_common.sh"
trap cleanup_tabs EXIT

fatal()
{
    echo "[FATAL] $1"
    echo "[FATAL] Startup stopped. Already opened nodes are left running for manual inspection."
    exit 1
}

check_duplicate_nodes()
{
    local node

    for node in \
        /mavros \
        /manager \
        /drone0/manager \
        /drone0/odomRemap \
        /drone0/ctrl_node
    do
        ensure_node_absent "$node" || return 1
    done

    echo "[OK] No duplicate MAVROS, manager, odomRemap or controller nodes."
}

check_real_time()
{
    local use_sim_time
    use_sim_time=$(rosparam get /use_sim_time 2>/dev/null || echo "false")

    if [ "$use_sim_time" = "true" ]; then
        echo "[ERROR] /use_sim_time=true. Real flight must not use simulation time."
        return 1
    fi

    echo "[OK] Real-flight wall clock is active."
}

check_px4_safe_state()
{
    local state
    local mode
    local armed

    state=$(timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null) ||
        return 1

    mode=$(echo "$state" |
        awk -F': ' '/^mode:/ {print $2}' |
        tr -d "'\"")
    armed=$(echo "$state" |
        awk -F': ' '/^armed:/ {print $2}')

    if [ "$armed" = "True" ]; then
        echo "[ERROR] PX4 is armed. Refusing to start or reconfigure the control chain."
        return 1
    fi

    if [ "$mode" = "OFFBOARD" ]; then
        echo "[ERROR] PX4 is already in OFFBOARD before the chain is ready."
        return 1
    fi

    echo "[OK] PX4 safe state: mode=${mode:-unknown}, armed=${armed:-unknown}."
}

check_vehicle_on_ground()
{
    local extended_state
    local landed_state

    extended_state=$(
        timeout 3 rostopic echo -n 1 /mavros/extended_state 2>/dev/null
    ) || {
        echo "[ERROR] Cannot read /mavros/extended_state."
        return 1
    }

    landed_state=$(echo "$extended_state" |
        awk -F': ' '/^[[:space:]]*landed_state:/ {print $2}')

    if [ "$landed_state" != "1" ]; then
        echo "[ERROR] PX4 does not report LANDED_STATE_ON_GROUND (landed_state=$landed_state)."
        return 1
    fi

    echo "[OK] PX4 reports that the vehicle is on the ground."
}

read_px4_param_field_once()
{
    local param_id="$1"
    local field="$2"
    local response
    local value

    response=$(
        timeout 4 rosservice call /mavros/param/get \
            "param_id: '${param_id}'" 2>/dev/null
    ) || return 1

    echo "$response" |
        grep -Eq '^[[:space:]]*success: (True|true)$' || return 1

    value=$(echo "$response" |
        awk -F': ' -v key="$field" '$1 ~ key "$" {print $2; exit}' |
        tr -d '[:space:]')
    [ -n "$value" ] || return 1
    printf '%s\n' "$value"
}

wait_px4_param_field()
{
    local param_id="$1"
    local field="$2"
    local timeout_s="${3:-20}"
    local start_time=$SECONDS
    local value

    while (( SECONDS - start_time < timeout_s )); do
        value=$(read_px4_param_field_once "$param_id" "$field") && {
            printf '%s\n' "$value"
            return 0
        }
        sleep 1
    done
    return 1
}

check_offboard_loss_failsafe()
{
    local loss_timeout
    local rc_action
    local no_rc_action

    # PX4 must remain independently capable of landing if this process or its
    # setpoint stream disappears before the ROS AUTO.LAND request is delivered.
    # Current PX4 releases use COM_OBL_RC_ACT; older releases additionally use
    # COM_OBL_ACT when no RC link is available. This check never writes params.
    [[ "$MAX_OFFBOARD_LOSS_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
        echo "[ERROR] Invalid MAX_OFFBOARD_LOSS_TIMEOUT=$MAX_OFFBOARD_LOSS_TIMEOUT."
        return 1
    }

    loss_timeout=$(wait_px4_param_field COM_OF_LOSS_T real 20) || {
        echo "[ERROR] Cannot read PX4 COM_OF_LOSS_T through MAVROS."
        return 1
    }
    [[ "$loss_timeout" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]] || {
        echo "[ERROR] PX4 COM_OF_LOSS_T is not numeric: $loss_timeout"
        return 1
    }
    awk -v value="$loss_timeout" -v maximum="$MAX_OFFBOARD_LOSS_TIMEOUT" \
        'BEGIN { exit !(value >= 0.0 && value <= maximum) }' || {
        echo "[ERROR] PX4 COM_OF_LOSS_T=$loss_timeout s exceeds the allowed ${MAX_OFFBOARD_LOSS_TIMEOUT} s."
        return 1
    }

    rc_action=$(wait_px4_param_field COM_OBL_RC_ACT integer 20) || {
        echo "[ERROR] Cannot read PX4 COM_OBL_RC_ACT through MAVROS."
        return 1
    }
    if [ "$rc_action" != "4" ]; then
        echo "[ERROR] PX4 COM_OBL_RC_ACT=$rc_action; HAM requires 4 (Land) for Offboard loss."
        return 1
    fi

    # COM_OBL_ACT was removed from newer PX4 releases. If present, require its
    # legacy no-RC action to be Land(0); absence is expected on newer firmware.
    if no_rc_action=$(read_px4_param_field_once COM_OBL_ACT integer); then
        if [ "$no_rc_action" != "0" ]; then
            echo "[ERROR] PX4 COM_OBL_ACT=$no_rc_action; legacy no-RC Offboard loss must be 0 (Land)."
            return 1
        fi
        echo "[OK] PX4 legacy no-RC Offboard-loss action is Land."
    else
        echo "[INFO] PX4 COM_OBL_ACT is not exposed; using the current COM_OBL_RC_ACT policy."
    fi

    echo "[OK] PX4 Offboard-loss failsafe: timeout=${loss_timeout}s, action=Land."
}

wait_rtk_fixed()
{
    local timeout_s="${1:-90}"
    local start_time=$SECONDS
    local gps_msg
    local fix_type

    echo "[WAIT] Waiting for RTK Fixed on $RTK_STATUS_TOPIC..."

    while (( SECONDS - start_time < timeout_s )); do
        gps_msg=$(
            timeout 2 rostopic echo -n 1 "$RTK_STATUS_TOPIC" 2>/dev/null
        )
        fix_type=$(echo "$gps_msg" |
            awk -F': ' '/^[[:space:]]*fix_type:/ {print $2; exit}')

        case "$fix_type" in
            ''|*[!0-9]*)
                ;;
            *)
                if [ "$fix_type" -ge "$RTK_FIXED_MIN_TYPE" ]; then
                    echo "[OK] RTK Fixed: fix_type=$fix_type."
                    return 0
                fi
                echo "[WAIT] Current GPS fix_type=$fix_type; RTK Fixed requires >=$RTK_FIXED_MIN_TYPE."
                ;;
        esac

        sleep 1
    done

    echo "[ERROR] RTK did not reach Fixed within ${timeout_s} s."
    echo "[ERROR] Verify that $RTK_STATUS_TOPIC is the GPSRAW topic used by this MAVROS installation."
    return 1
}

wait_ekf_ready()
{
    local timeout_s="${1:-90}"
    local start_time=$SECONDS
    local estimator

    echo "[WAIT] Waiting for PX4 EKF absolute-position readiness..."

    while (( SECONDS - start_time < timeout_s )); do
        estimator=$(
            timeout 2 rostopic echo -n 1 /mavros/estimator_status 2>/dev/null
        )

        if echo "$estimator" |
            grep -Eq "pos_horiz_abs_status_flag: (True|true)" &&
           echo "$estimator" |
            grep -Eq "pos_vert_abs_status_flag: (True|true)" &&
           ! echo "$estimator" |
            grep -Eq "gps_glitch_status_flag: (True|true)"; then
            echo "[OK] PX4 EKF absolute position is ready."
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] PX4 EKF was not ready within ${timeout_s} s."
    echo "[ERROR] Check /mavros/estimator_status and the PX4 estimator configuration."
    return 1
}

check_fcu_device_access()
{
    local mode
    local other_digit

    [ -e "$FCU_DEVICE" ] || {
        echo "[ERROR] FCU device does not exist: $FCU_DEVICE"
        return 1
    }

    [ -r "$FCU_DEVICE" ] && [ -w "$FCU_DEVICE" ] || {
        echo "[ERROR] Current user cannot read/write $FCU_DEVICE."
        echo "[ERROR] Add the user to the dialout group and install a udev rule with MODE=\"0660\"."
        return 1
    }

    mode=$(stat -c '%a' "$FCU_DEVICE" 2>/dev/null) || return 1
    other_digit=${mode: -1}
    if (( (other_digit & 2) != 0 )); then
        echo "[ERROR] $FCU_DEVICE is world-writable (mode=$mode)."
        echo "[ERROR] Refusing insecure FCU access; use group dialout with mode 0660."
        return 1
    fi

    echo "[OK] FCU device access is available without world-write permission (mode=$mode)."
}

echo "================ HAM real-flight startup ================"

# 1. USB permission must be provisioned by dialout/udev; never make the FCU world-writable.
check_fcu_device_access ||
    fatal "Secure access to $FCU_DEVICE is not configured"

# 2. Start or reuse ROS master.
ensure_roscore ||
    fatal "ROS master is unavailable"

check_real_time ||
    fatal "Simulation time is enabled"

check_duplicate_nodes ||
    fatal "A previous HAM flight-control chain is still running"

if fuser "$FCU_DEVICE" >/dev/null 2>&1; then
    fatal "$FCU_DEVICE is already in use, possibly by another MAVROS process"
fi

# 3. Connect MAVROS to Pixhawk.
T_MAVROS=$(mk_tab "roslaunch mavros px4.launch fcu_url:=serial://$FCU_DEVICE:$FCU_BAUD")
gnome-terminal --tab --title="mavros" -e "$T_MAVROS"

wait_mavros_connected 120 "Check $FCU_DEVICE, baud $FCU_BAUD, Pixhawk power and firmware." ||
    fatal "MAVROS did not connect to Pixhawk"

# 4. Configure required MAVLink data before the extended-state ground check,
# odomRemap and controller startup.
# 实飞频率: ID105=200Hz(默认), ID31/ID32=100Hz, ID331=50Hz,
# ID245=2Hz, GPS_RAW_INT/ESTIMATOR_STATUS=5Hz(默认)。
ENABLE_NAV_STATUS_RATES=1 ID31=10000 ID32=10000 ID331=20000 bash "$SCRIPT_DIR/mavlink_set_rates.sh" ||
    fatal "MAVLink message interval configuration failed"

check_offboard_loss_failsafe ||
    fatal "PX4 Offboard-loss failsafe is not configured for timely landing"

check_px4_safe_state ||
    fatal "PX4 is not in a safe startup state"

check_vehicle_on_ground ||
    fatal "PX4 does not confirm that the vehicle is on the ground"

wait_topic_messages /mavros/imu/data 2 30 ||
    fatal "MAVROS IMU data is unavailable"

wait_rtk_fixed 90 ||
    fatal "RTK Fixed is unavailable"

wait_ekf_ready 90 ||
    fatal "PX4 EKF is not ready"

wait_topic_messages /mavros/local_position/odom 2 30 ||
    fatal "MAVROS local-position odometry is unavailable"

wait_topic_messages /mavros/local_position/velocity_local 2 30 ||
    fatal "MAVROS local velocity is unavailable"

# 5. Start trajectory node and create /manager plus /drone0/manager.
T_TRAJ=$(mk_tab "roslaunch trajectory_utils traj_gen.launch")
gnome-terminal --tab --title="traj_gen" -e "$T_TRAJ"

wait_node /manager 60 ||
    fatal "Global nodelet manager /manager did not start"

wait_node /drone0/manager 60 ||
    fatal "Nodelet manager /drone0/manager did not start"

# 6. Start odomRemap and wait for calibration plus actual output data.
T_REMAP=$(mk_tab "roslaunch realflight_utils odom_remap.launch")
gnome-terminal --tab --title="odom_remap" -e "$T_REMAP"

wait_nodelet /drone0/manager odomRemap 60 ||
    fatal "odomRemap failed to load"

wait_topic_messages /odom/remap 2 30 ||
    fatal "odomRemap loaded, but calibration/output did not become ready"

# Recheck state immediately before loading the controller.
check_px4_safe_state ||
    fatal "PX4 state changed before controller startup"

# 7. Load ctrl_node only after /odom/remap is valid.
T_CTRL=$(mk_tab "roslaunch ctrl_node run_ctrl.launch navigation_monitor_enabled:=true")
gnome-terminal --tab --title="controller" -e "$T_CTRL"

wait_nodelet /drone0/manager ctrl_node 60 ||
    fatal "ctrl_node failed to load"

wait_topic_messages /mavros/setpoint_raw/attitude 2 15 ||
    fatal "ctrl_node loaded but is not publishing attitude setpoints"

check_px4_safe_state ||
    fatal "PX4 unexpectedly armed or entered OFFBOARD during startup"

echo "======================= READY ============================"
echo "[READY] MAVROS connected to Pixhawk."
echo "[READY] RTK Fixed and PX4 EKF absolute position are valid."
echo "[READY] /odom/remap is calibrated and publishing."
echo "[READY] ctrl_node is publishing attitude setpoints."
echo "[SAFE]  Vehicle remains disarmed and is not in OFFBOARD."
echo "[NEXT]  After manual inspection, run set_offboard.sh and takeoff.sh separately."
echo "==========================================================="
