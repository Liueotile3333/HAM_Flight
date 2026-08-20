#!/usr/bin/env bash
# HAM 室外实飞：录包 -> 人工切 OFFBOARD -> 起飞 -> 稳定悬停
#              -> 轨迹 -> 静止 -> 降落 -> 确认上锁
#
# 重要安全约束：
#   1. 本脚本不切换 OFFBOARD，模式只能由连接 Pixhawk 6C 的外部遥控器切换。
#   2. 本脚本不使用固定 sleep 判断飞行阶段，只根据 ROS/PX4 状态推进。
#   3. 空中异常或 Ctrl-C 时，不关闭 MAVROS、odomRemap、规划器或控制器，便于遥控器接管。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly HAM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly UTILS_DIR="${SCRIPT_DIR}"
readonly RECORD_SCRIPT="${UTILS_DIR}/record.sh"

readonly STATE_TOPIC="/mavros/state"
readonly EXTENDED_STATE_TOPIC="/mavros/extended_state"
readonly ESTIMATOR_TOPIC="/mavros/estimator_status"
readonly RTK_STATUS_TOPIC="/mavros/gpsstatus/gps1/raw"
readonly ODOM_TOPIC="/odom/remap"
readonly SETPOINT_TOPIC="/mavros/setpoint_raw/attitude"
readonly TAKEOFF_LAND_TOPIC="/drone0/ctrl_node/takeoff_land"
readonly TRAJECTORY_TRIGGER_TOPIC="/landing_trigger"
readonly TRAJECTORY_COMMAND_TOPIC="/drone0/planning/cmd"
readonly TRAJECTORY_TIME_TOPIC="/drone0/sim_odom/time_diff"
readonly TRAJECTORY_PARAM_NS="/drone0/sim_odom"
readonly CONTROL_PARAM_NS="/drone0/ctrl_node"

readonly RTK_FIXED_MIN_TYPE=6
readonly OFFBOARD_WAIT_TIMEOUT=60
readonly ARM_WAIT_TIMEOUT=20
readonly HOVER_WAIT_TIMEOUT=45
readonly TRAJECTORY_START_TIMEOUT=15
readonly LAND_WAIT_TIMEOUT=120
readonly MIN_FREE_GB="${MIN_FREE_GB:-5}"

RECORD_PID=""
RECORD_LOG=""
AIRBORNE=0

exec 9>/tmp/ham_whole_traj.lock
if ! flock -n 9; then
    echo "[FATAL] 另一个 whole_traj.sh 正在运行，拒绝重复执行。" >&2
    exit 1
fi

# 公共 ROS 工具(log/read_state/state_value/wait_topic_messages 等)统一取自 _ros_common.sh,
# 避免与本脚本出现两份逐渐分叉的实现。
source "${UTILS_DIR}/_ros_common.sh"

fatal()
{
    log FATAL "$1" >&2
    if (( AIRBORNE )); then
        log SAFETY "飞行器可能仍在空中：停止自动流程，但保留 MAVROS、控制器、里程计和录包。" >&2
        log SAFETY "请立即通过外部遥控器退出 OFFBOARD，并按现场预案接管/降落。" >&2
    fi
    exit 1
}

stop_recording()
{
    if [[ -z "$RECORD_PID" ]] || ! kill -0 "$RECORD_PID" 2>/dev/null; then
        return 0
    fi

    log INFO "正在停止 rosbag（进程组 ${RECORD_PID}）..."
    kill -INT -- "-${RECORD_PID}" 2>/dev/null || true

    local start_time=$SECONDS
    while kill -0 "$RECORD_PID" 2>/dev/null; do
        if (( SECONDS - start_time >= 15 )); then
            log WARN "rosbag 未在 15 s 内退出，请检查进程组 ${RECORD_PID}。"
            return 1
        fi
        sleep 1
    done

    wait "$RECORD_PID" 2>/dev/null || true
    log OK "rosbag 已停止并完成索引写入。"
}

cleanup()
{
    local exit_code=$?

    if [[ -n "$RECORD_PID" ]] && kill -0 "$RECORD_PID" 2>/dev/null; then
        if (( AIRBORNE )); then
            log WARN "脚本在空中阶段结束，保留 rosbag 继续记录。录包进程组：${RECORD_PID}。" >&2
            [[ -n "$RECORD_LOG" ]] && log WARN "录包日志：${RECORD_LOG}" >&2
        else
            stop_recording || true
        fi
    fi

    return "$exit_code"
}

on_signal()
{
    log WARN "收到中断信号，停止任务自动推进。" >&2
    if (( AIRBORNE )); then
        log SAFETY "不会自动降落或上锁；请使用外部遥控器接管。" >&2
    fi
    exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

require_command()
{
    command -v "$1" >/dev/null 2>&1 || fatal "缺少命令：$1"
}

is_number()
{
    [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]
}

read_vector()
{
    local topic="$1"
    local msg x y z

    msg=$(timeout 3 rostopic echo -n 1 "$topic" 2>/dev/null) || return 1
    x=$(awk -F': ' '$1 == "x" {print $2; exit}' <<<"$msg")
    y=$(awk -F': ' '$1 == "y" {print $2; exit}' <<<"$msg")
    z=$(awk -F': ' '$1 == "z" {print $2; exit}' <<<"$msg")

    is_number "$x" && is_number "$y" && is_number "$z" || return 1
    printf '%s %s %s\n' "$x" "$y" "$z"
}

read_position()
{
    read_vector "${ODOM_TOPIC}/pose/pose/position"
}

read_velocity()
{
    read_vector "${ODOM_TOPIC}/twist/twist/linear"
}

read_scalar_topic()
{
    local topic="$1"
    local msg value

    msg=$(timeout 3 rostopic echo -n 1 "$topic" 2>/dev/null) || return 1
    value=$(awk -F': ' '$1 == "data" {print $2; exit}' <<<"$msg")
    is_number "$value" || return 1
    printf '%s\n' "$value"
}

topic_has_subscriber()
{
    local topic="$1"
    local info
    info=$(rostopic info "$topic" 2>/dev/null) || return 1
    awk '
        /^Subscribers:/ {inside=1; next}
        /^(Publishers|Services):/ {inside=0}
        inside && /^[[:space:]]*\*/ {found=1}
        END {exit !found}
    ' <<<"$info"
}

assert_nodelet()
{
    local manager="$1"
    local nodelet="$2"

    rosservice call "${manager}/list" 2>/dev/null | grep -Fq "$nodelet" ||
        fatal "节点组件未加载：${nodelet}（manager=${manager}）"
}

assert_safe_preflight_state()
{
    local state mode armed connected
    state=$(read_state "$STATE_TOPIC") || fatal "无法读取 ${STATE_TOPIC}"
    connected=$(state_value "$state" connected)
    armed=$(state_value "$state" armed)
    mode=$(state_value "$state" mode)

    [[ "$connected" == "True" ]] || fatal "MAVROS 尚未连接 Pixhawk 6C。"
    [[ "$armed" == "False" ]] || fatal "飞行器已经解锁，拒绝启动任务。"
    [[ "$mode" != "OFFBOARD" ]] || fatal "任务启动前已处于 OFFBOARD；请先退出 OFFBOARD 并保持未解锁。"

    local extended landed_state
    extended=$(timeout 3 rostopic echo -n 1 "$EXTENDED_STATE_TOPIC" 2>/dev/null) ||
        fatal "无法读取 ${EXTENDED_STATE_TOPIC}"
    landed_state=$(awk -F': ' '/^[[:space:]]*landed_state:/ {print $2; exit}' <<<"$extended")
    [[ "$landed_state" == "1" ]] || fatal "PX4 未报告 ON_GROUND（landed_state=${landed_state:-unknown}）。"

    log OK "PX4 初始状态安全：connected=True, armed=False, mode=${mode:-unknown}, ON_GROUND。"
}

assert_rtk_fixed()
{
    local msg fix_type
    msg=$(timeout 3 rostopic echo -n 1 "$RTK_STATUS_TOPIC" 2>/dev/null) ||
        fatal "无法读取 RTK 状态：${RTK_STATUS_TOPIC}"
    fix_type=$(awk -F': ' '/^[[:space:]]*fix_type:/ {print $2; exit}' <<<"$msg")

    [[ "$fix_type" =~ ^[0-9]+$ ]] || fatal "RTK fix_type 无效：${fix_type:-missing}"
    (( fix_type >= RTK_FIXED_MIN_TYPE )) || fatal "RTK 不是 Fixed：fix_type=${fix_type}，要求 >=${RTK_FIXED_MIN_TYPE}。"
    log OK "RTK Fixed：fix_type=${fix_type}。"
}

assert_ekf_ready()
{
    local estimator
    estimator=$(timeout 3 rostopic echo -n 1 "$ESTIMATOR_TOPIC" 2>/dev/null) ||
        fatal "无法读取 PX4 EKF 状态：${ESTIMATOR_TOPIC}"

    grep -Eq 'pos_horiz_abs_status_flag: (True|true)' <<<"$estimator" ||
        fatal "PX4 EKF 水平绝对位置未就绪。"
    grep -Eq 'pos_vert_abs_status_flag: (True|true)' <<<"$estimator" ||
        fatal "PX4 EKF 垂直绝对位置未就绪。"
    if grep -Eq 'gps_glitch_status_flag: (True|true)' <<<"$estimator"; then
        fatal "PX4 EKF 报告 GPS glitch。"
    fi

    log OK "PX4 EKF 绝对位置状态正常。"
}

check_disk_space()
{
    local free_kb required_kb
    [[ "$MIN_FREE_GB" =~ ^[0-9]+$ ]] || fatal "MIN_FREE_GB 必须是正整数。"
    (( MIN_FREE_GB > 0 )) || fatal "MIN_FREE_GB 必须大于 0。"
    free_kb=$(df -Pk "$UTILS_DIR" | awk 'NR == 2 {print $4}')
    required_kb=$(( MIN_FREE_GB * 1024 * 1024 ))

    [[ "$free_kb" =~ ^[0-9]+$ ]] || fatal "无法读取录包磁盘剩余空间。"
    (( free_kb >= required_kb )) ||
        fatal "录包磁盘空间不足：至少需要 ${MIN_FREE_GB} GiB。"

    log OK "录包磁盘可用空间满足最低要求 ${MIN_FREE_GB} GiB。"
}

start_recording()
{
    local stamp
    stamp=$(date +%Y%m%d_%H%M%S)
    RECORD_LOG="${UTILS_DIR}/rosbag_${stamp}.log"

    log INFO "启动录包：${RECORD_SCRIPT}"
    (
        cd "$UTILS_DIR"
        exec setsid bash "$RECORD_SCRIPT"
    ) >"$RECORD_LOG" 2>&1 &
    RECORD_PID=$!

    sleep 2
    kill -0 "$RECORD_PID" 2>/dev/null || fatal "rosbag 启动失败，请查看 ${RECORD_LOG}"

    if ! rosnode list 2>/dev/null | grep -Eq '^/record(_[0-9]+)?$'; then
        fatal "未发现 rosbag record 节点，请查看 ${RECORD_LOG}"
    fi

    log OK "rosbag 正在运行，进程组=${RECORD_PID}，日志=${RECORD_LOG}。"
}

wait_for_external_offboard()
{
    local start_time=$SECONDS
    local state mode armed connected

    log ACTION "请确认起飞区域和遥控器接管开关安全，然后用外部遥控器切入 OFFBOARD。"
    log WAIT "等待 PX4 进入 OFFBOARD（最长 ${OFFBOARD_WAIT_TIMEOUT} s）..."

    while (( SECONDS - start_time < OFFBOARD_WAIT_TIMEOUT )); do
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        armed=$(state_value "$state" armed)
        mode=$(state_value "$state" mode)

        [[ "$connected" == "True" ]] || fatal "等待 OFFBOARD 时 MAVROS 连接断开。"
        [[ "$armed" == "False" ]] || fatal "发送起飞命令前飞行器意外解锁。"

        if [[ "$mode" == "OFFBOARD" ]]; then
            log OK "外部遥控器已切入 OFFBOARD，飞行器仍未解锁。"
            return 0
        fi
        sleep 1
    done

    fatal "等待外部遥控器切入 OFFBOARD 超时。"
}

publish_takeoff()
{
    topic_has_subscriber "$TAKEOFF_LAND_TOPIC" ||
        fatal "${TAKEOFF_LAND_TOPIC} 没有 ctrl_node 订阅者。"

    log ACTION "发送一次起飞命令。"
    rostopic pub -1 "$TAKEOFF_LAND_TOPIC" quadrotor_msgs/TakeoffLand \
        "{takeoff_land_cmd: 1}" >/dev/null || fatal "起飞命令发布失败。"
}

wait_for_armed()
{
    local start_time=$SECONDS
    local state mode armed connected

    log WAIT "等待 PX4 解锁（最长 ${ARM_WAIT_TIMEOUT} s）..."
    while (( SECONDS - start_time < ARM_WAIT_TIMEOUT )); do
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        armed=$(state_value "$state" armed)
        mode=$(state_value "$state" mode)

        [[ "$connected" == "True" ]] || fatal "等待解锁时 MAVROS 连接断开。"
        [[ "$mode" == "OFFBOARD" ]] || fatal "等待解锁时已经退出 OFFBOARD。"

        if [[ "$armed" == "True" ]]; then
            AIRBORNE=1
            log OK "PX4 已解锁，起飞阶段开始。"
            return 0
        fi
        sleep 1
    done

    fatal "PX4 未在 ${ARM_WAIT_TIMEOUT} s 内解锁；起飞命令可能被拒绝。"
}

wait_for_stable_hover()
{
    local initial_z="$1"
    local takeoff_height target_z
    local stable_count=0
    local start_time=$SECONDS

    takeoff_height=$(rosparam get "${CONTROL_PARAM_NS}/takeoff_state/height" 2>/dev/null) ||
        fatal "缺少起飞高度参数 ${CONTROL_PARAM_NS}/takeoff_state/height"
    is_number "$takeoff_height" || fatal "起飞高度参数不是数字：${takeoff_height}"
    target_z=$(awk -v z="$initial_z" -v h="$takeoff_height" 'BEGIN {printf "%.6f", z + h}')

    log WAIT "等待稳定悬停：目标 z≈${target_z} m（最长 ${HOVER_WAIT_TIMEOUT} s）..."

    while (( SECONDS - start_time < HOVER_WAIT_TIMEOUT )); do
        local state mode armed position velocity
        local px py pz vx vy vz metrics position_error speed

        state=$(read_state "$STATE_TOPIC") || fatal "悬停等待期间无法读取 PX4 状态。"
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)
        [[ "$mode" == "OFFBOARD" && "$armed" == "True" ]] ||
            fatal "悬停完成前状态异常：mode=${mode:-unknown}, armed=${armed:-unknown}。"

        position=$(read_position) || fatal "悬停等待期间 ${ODOM_TOPIC} 位置数据中断。"
        velocity=$(read_velocity) || fatal "悬停等待期间 ${ODOM_TOPIC} 速度数据中断。"
        read -r px py pz <<<"$position"
        read -r vx vy vz <<<"$velocity"

        metrics=$(awk -v z="$pz" -v tz="$target_z" -v x="$vx" -v y="$vy" -v vz="$vz" \
            'BEGIN {e=z-tz; if(e<0)e=-e; printf "%.6f %.6f", e, sqrt(x*x+y*y+vz*vz)}')
        read -r position_error speed <<<"$metrics"

        if awk -v e="$position_error" -v v="$speed" 'BEGIN {exit !(e <= 0.25 && v <= 0.20)}'; then
            ((stable_count += 1))
            if (( stable_count >= 10 )); then
                log OK "稳定悬停确认：|z-z_target|<=0.25 m 且速度<=0.20 m/s，连续 10 帧。"
                return 0
            fi
        else
            stable_count=0
        fi

        sleep 0.2
    done

    fatal "未在 ${HOVER_WAIT_TIMEOUT} s 内达到稳定悬停。"
}

trajectory_total_time()
{
    local rise duration hold descend scale
    rise=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_rise_time" 2>/dev/null) || return 1
    duration=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_duration" 2>/dev/null) || return 1
    hold=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_hold_time" 2>/dev/null) || return 1
    descend=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_descend_time" 2>/dev/null) || return 1
    scale=$(rosparam get "${TRAJECTORY_PARAM_NS}/speed_scale" 2>/dev/null) || return 1

    is_number "$rise" && is_number "$duration" && is_number "$hold" &&
        is_number "$descend" && is_number "$scale" || return 1
    awk -v a="$rise" -v b="$duration" -v c="$hold" -v d="$descend" -v s="$scale" \
        'BEGIN {if(s<=0) exit 1; printf "%.6f", (a+b+c+d)/s}'
}

publish_trajectory_trigger()
{
    topic_has_subscriber "$TRAJECTORY_TRIGGER_TOPIC" ||
        fatal "${TRAJECTORY_TRIGGER_TOPIC} 没有轨迹规划器订阅者。"

    log ACTION "发布一次轨迹触发。"
    rostopic pub -1 "$TRAJECTORY_TRIGGER_TOPIC" geometry_msgs/PoseStamped \
        "{header: {stamp: now, frame_id: ''}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
        >/dev/null || fatal "轨迹触发发布失败。"
}

wait_for_trajectory_start()
{
    local start_time=$SECONDS
    local elapsed

    log WAIT "等待规划器接受触发并从 t≈0 开始发布轨迹..."
    while (( SECONDS - start_time < TRAJECTORY_START_TIMEOUT )); do
        elapsed=$(read_scalar_topic "$TRAJECTORY_TIME_TOPIC" 2>/dev/null || true)
        if [[ -n "$elapsed" ]] &&
           awk -v t="$elapsed" 'BEGIN {exit !(t >= 0.0 && t <= 2.0)}'; then
            wait_topic_messages "$TRAJECTORY_COMMAND_TOPIC" 3 5 ||
                fatal "轨迹时间已经启动，但 ${TRAJECTORY_COMMAND_TOPIC} 未连续发布。"
            log OK "轨迹已启动：t=${elapsed} s。"
            return 0
        fi
        sleep 0.2
    done

    fatal "规划器未确认轨迹启动；触发可能因无 odom、速度过大或旧轨迹仍活动而被拒绝。"
}

wait_for_trajectory_finish()
{
    local total="$1"
    local timeout_s last_progress_time last_elapsed
    local start_time=$SECONDS
    timeout_s=$(awk -v t="$total" 'BEGIN {printf "%d", t + 45}')
    last_progress_time=$SECONDS
    last_elapsed="-1"

    log WAIT "轨迹执行中：运行时参数计算总时长=${total} s，超时=${timeout_s} s。"

    while (( SECONDS - start_time < timeout_s )); do
        local state mode armed elapsed
        state=$(read_state "$STATE_TOPIC") || fatal "轨迹期间无法读取 PX4 状态。"
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)
        [[ "$mode" == "OFFBOARD" && "$armed" == "True" ]] ||
            fatal "轨迹期间已经退出 OFFBOARD 或上锁：mode=${mode:-unknown}, armed=${armed:-unknown}。"

        wait_topic_messages "$ODOM_TOPIC" 2 3 || fatal "轨迹期间 ${ODOM_TOPIC} 数据中断。"
        elapsed=$(read_scalar_topic "$TRAJECTORY_TIME_TOPIC" 2>/dev/null || true)
        [[ -n "$elapsed" ]] || fatal "轨迹时间话题 ${TRAJECTORY_TIME_TOPIC} 中断。"

        if awk -v now="$elapsed" -v old="$last_elapsed" 'BEGIN {exit !(now > old + 0.05)}'; then
            last_elapsed="$elapsed"
            last_progress_time=$SECONDS
        elif (( SECONDS - last_progress_time >= 5 )); then
            fatal "轨迹时钟连续 5 s 未推进（当前 t=${elapsed} s）。"
        fi

        if awk -v now="$elapsed" -v end="$total" 'BEGIN {exit !(now >= end)}'; then
            log OK "轨迹时间已完成：t=${elapsed} s / ${total} s。"
            return 0
        fi

        log INFO "轨迹进度：${elapsed} / ${total} s"
        sleep 1
    done

    fatal "轨迹未在 ${timeout_s} s 内完成。"
}

wait_for_stationary()
{
    local stable_count=0
    local start_time=$SECONDS

    log WAIT "等待降落前静止：水平速度<=0.08 m/s、垂直速度<=0.10 m/s，连续 15 帧。"
    while (( SECONDS - start_time < 30 )); do
        local state mode armed velocity vx vy vz metrics horizontal vertical
        state=$(read_state "$STATE_TOPIC") || fatal "降落前无法读取 PX4 状态。"
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)
        [[ "$mode" == "OFFBOARD" && "$armed" == "True" ]] ||
            fatal "降落前状态异常：mode=${mode:-unknown}, armed=${armed:-unknown}。"

        velocity=$(read_velocity) || fatal "降落前 ${ODOM_TOPIC} 速度数据中断。"
        read -r vx vy vz <<<"$velocity"
        metrics=$(awk -v x="$vx" -v y="$vy" -v z="$vz" \
            'BEGIN {if(z<0)z=-z; printf "%.6f %.6f", sqrt(x*x+y*y), z}')
        read -r horizontal vertical <<<"$metrics"

        if awk -v h="$horizontal" -v v="$vertical" 'BEGIN {exit !(h<=0.08 && v<=0.10)}'; then
            ((stable_count += 1))
            if (( stable_count >= 15 )); then
                log OK "降落前静止条件满足。"
                return 0
            fi
        else
            stable_count=0
        fi
        sleep 0.2
    done

    fatal "轨迹完成后 30 s 内仍未满足安全降落速度门限。"
}

publish_land()
{
    topic_has_subscriber "$TAKEOFF_LAND_TOPIC" ||
        fatal "${TAKEOFF_LAND_TOPIC} 没有 ctrl_node 订阅者。"

    log ACTION "发送一次降落命令；控制器将立即请求 PX4 AUTO.LAND。"
    rostopic pub -1 "$TAKEOFF_LAND_TOPIC" quadrotor_msgs/TakeoffLand \
        "{takeoff_land_cmd: 2}" >/dev/null || fatal "降落命令发布失败。"
}

confirm_landing_started()
{
    local start_z="$1"
    local start_time=$SECONDS

    log WAIT "确认控制器已接受降落命令..."
    while (( SECONDS - start_time < 12 )); do
        local state mode armed position px py pz
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)

        if [[ "$armed" == "False" || "$mode" == "AUTO.LAND" ]]; then
            log OK "PX4 已进入降落/上锁阶段：mode=${mode:-unknown}, armed=${armed:-unknown}。"
            return 0
        fi

        if [[ "$mode" != "OFFBOARD" ]]; then
            fatal "确认降落期间进入非预期模式 ${mode:-unknown}；视为遥控器接管。"
        fi

        position=$(read_position 2>/dev/null || true)
        if [[ -n "$position" ]]; then
            read -r px py pz <<<"$position"
            if awk -v z="$pz" -v z0="$start_z" 'BEGIN {exit !(z <= z0 - 0.05)}'; then
                log OK "检测到下降：z=${pz} m，降落命令已生效。"
                return 0
            fi
        fi
        sleep 1
    done

    return 1
}

wait_for_disarmed()
{
    local start_time=$SECONDS
    local last_mode=""

    log WAIT "等待控制器降落、PX4 AUTO.LAND 和自动上锁（最长 ${LAND_WAIT_TIMEOUT} s）..."
    while (( SECONDS - start_time < LAND_WAIT_TIMEOUT )); do
        local state connected mode armed
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)

        [[ "$connected" == "True" ]] || fatal "降落期间 MAVROS 连接断开。"

        if [[ "$mode" != "$last_mode" ]]; then
            log INFO "降落阶段 PX4 mode=${mode:-unknown}, armed=${armed:-unknown}。"
            last_mode="$mode"
        fi

        if [[ "$armed" == "False" ]]; then
            AIRBORNE=0
            log OK "PX4 已上锁。"
            return 0
        fi

        if [[ "$mode" != "OFFBOARD" && "$mode" != "AUTO.LAND" ]]; then
            fatal "降落期间进入非预期模式 ${mode:-unknown}；视为遥控器接管，停止自动流程。"
        fi
        sleep 1
    done

    fatal "降落后 ${LAND_WAIT_TIMEOUT} s 内 PX4 仍未上锁。"
}

wait_until_on_ground()
{
    local start_time=$SECONDS
    while (( SECONDS - start_time < 20 )); do
        local extended landed_state
        extended=$(timeout 3 rostopic echo -n 1 "$EXTENDED_STATE_TOPIC" 2>/dev/null || true)
        landed_state=$(awk -F': ' '/^[[:space:]]*landed_state:/ {print $2; exit}' <<<"$extended")
        if [[ "$landed_state" == "1" ]]; then
            log OK "PX4 确认 LANDED_STATE_ON_GROUND。"
            return 0
        fi
        sleep 1
    done
    log WARN "PX4 已上锁，但 20 s 内未确认 ON_GROUND；请人工检查 landed_state。"
}

main()
{
    echo "================ HAM 实飞轨迹任务 ================"

    for command in flock timeout rostopic rosnode rosservice rosparam awk grep df setsid; do
        require_command "$command"
    done

    [[ -d "$HAM_ROOT" ]] || fatal "工程目录不存在：${HAM_ROOT}"
    [[ -d "$UTILS_DIR" ]] || fatal "脚本目录不存在：${UTILS_DIR}"
    [[ -f "$RECORD_SCRIPT" ]] || fatal "录包脚本不存在：${RECORD_SCRIPT}"

    rostopic list >/dev/null 2>&1 || fatal "ROS master 不可用；请先运行修复后的 RTK.sh。"
    [[ "$(rosparam get /use_sim_time 2>/dev/null || echo false)" != "true" ]] ||
        fatal "实飞禁止 /use_sim_time=true。"

    assert_nodelet /manager sim_odom
    assert_nodelet /drone0/manager odomRemap
    assert_nodelet /drone0/manager ctrl_node
    wait_topic_messages "$ODOM_TOPIC" 5 5 || fatal "${ODOM_TOPIC} 未连续发布。"
    wait_topic_messages "$SETPOINT_TOPIC" 5 5 || fatal "控制器未连续发布 OFFBOARD setpoint。"
    topic_has_subscriber "$TRAJECTORY_TRIGGER_TOPIC" ||
        fatal "轨迹触发话题没有 sim_odom 订阅者。"

    assert_safe_preflight_state
    assert_rtk_fixed
    assert_ekf_ready
    check_disk_space

    local initial_position initial_x initial_y initial_z total_time confirmation
    local traj_type traj_radius traj_amp_y traj_height path_scale speed_scale
    initial_position=$(read_position) || fatal "无法读取起飞前 ${ODOM_TOPIC} 位置。"
    read -r initial_x initial_y initial_z <<<"$initial_position"
    total_time=$(trajectory_total_time) || fatal "无法从运行时 ROS 参数计算轨迹总时长。"
    traj_type=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_type" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 traj_type。"
    traj_radius=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_radius" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 traj_radius。"
    traj_amp_y=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_amp_y" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 traj_amp_y。"
    traj_height=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_height" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 traj_height。"
    path_scale=$(rosparam get "${TRAJECTORY_PARAM_NS}/path_scale" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 path_scale。"
    speed_scale=$(rosparam get "${TRAJECTORY_PARAM_NS}/speed_scale" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 speed_scale。"
    local navigation_monitor_enabled
    navigation_monitor_enabled=$(rosparam get "${CONTROL_PARAM_NS}/navigation_monitor/enabled" 2>/dev/null) ||
        fatal "缺少 RTK/EKF 持续监测参数 navigation_monitor/enabled。"
    [[ "$navigation_monitor_enabled" == "true" || "$navigation_monitor_enabled" == "True" ]] ||
        fatal "RTK 实飞禁止关闭 navigation_monitor/enabled；请使用 RTK 启动脚本加载控制器。"

    echo "---------------------------------------------------"
    echo "起飞前位置：x=${initial_x}, y=${initial_y}, z=${initial_z} m"
    echo "轨迹空间参数：type=${traj_type}, radius=${traj_radius} m, amp_y=${traj_amp_y} m, height=${traj_height} m"
    echo "轨迹缩放参数：path_scale=${path_scale}, speed_scale=${speed_scale}"
    echo "轨迹总时长：${total_time} s（读取运行时参数，不使用固定 55 s）"
    echo "脚本不会切换 OFFBOARD；进入 OFFBOARD 后将发送起飞命令并由 FSM 请求解锁。"
    echo "确认桨区清空、遥控器接管开关有效、RTK/EKF 正常后，输入 TAKEOFF。"
    read -r -p "> " confirmation || fatal "未获得起飞确认。"
    [[ "$confirmation" == "TAKEOFF" ]] || fatal "未输入 TAKEOFF，任务取消。"

    start_recording
    wait_for_external_offboard
    publish_takeoff
    wait_for_armed
    wait_for_stable_hover "$initial_z"

    # 起飞过程中仍可能发生 RTK/EKF 状态变化；进入大范围水平轨迹前再次确认。
    assert_rtk_fixed
    assert_ekf_ready

    publish_trajectory_trigger
    wait_for_trajectory_start
    wait_for_trajectory_finish "$total_time"
    wait_for_stationary

    local preland_position preland_x preland_y preland_z
    preland_position=$(read_position) || fatal "发送降落命令前无法读取当前位置。"
    read -r preland_x preland_y preland_z <<<"$preland_position"

    publish_land
    if ! confirm_landing_started "$preland_z"; then
        log WARN "第一次降落命令在 12 s 内未观察到下降；重新确认静止后仅重试一次。"
        wait_for_stationary
        publish_land
        confirm_landing_started "$preland_z" ||
            fatal "第二次降落命令仍未生效；停止自动重试，请用遥控器接管。"
    fi
    wait_for_disarmed
    wait_until_on_ground

    echo "================ TASK COMPLETE ===================="
    log OK "轨迹、降落和自动上锁全部完成。"
    log INFO "脚本退出时将正常停止 rosbag；MAVROS、规划器、odomRemap 和控制器保持运行。"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
