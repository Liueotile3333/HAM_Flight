#!/bin/bash
# =============================================================================
# _ros_common.sh —— ROS 启动编排公共工具函数
# 供 sh_utils/ 下各启动脚本 source 复用。仅含无业务副作用的纯函数;
# 带业务语义(AIRBORNE 安全门禁、固定话题名门禁等)的函数由各脚本自行实现。
# 实现取自 RTK_realflight_traj_follow.sh 的实飞验证版本。
#
# 用法:source "$HAM/sh_utils/_ros_common.sh"
# =============================================================================
[ "${_ROS_COMMON_SOURCED:-0}" = 1 ] && return 0 2>/dev/null || true
_ROS_COMMON_SOURCED=1

# 本公共库自身绝对路径(mk_tab 生成的 tab 脚本要 source 它)。
_ROS_COMMON_SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# ---------- 日志 / 退出 ----------

log()
{
    printf '[%s] %s\n' "$1" "$2"
}

die()
{
    log ERROR "$1" >&2
    exit 1
}

# ---------- 等待就绪(轮询,带超时) ----------

# 等待独立 rosnode 出现。注意:不适用于 nodelet —— nodelet 不在 rosnode list 中,
# 请改用 wait_nodelet。
wait_node()
{
    local node="$1"
    local timeout_s="${2:-60}"
    local start_time=$SECONDS

    log WAIT "等待节点 ${node}..."
    until rosnode list 2>/dev/null | grep -qx "$node"; do
        sleep 1
        (( SECONDS - start_time < timeout_s )) || {
            log ERROR "节点 ${node} 未在 ${timeout_s}s 内就绪。"
            return 1
        }
    done
    log OK "节点 ${node} 就绪。"
}

# 等待 nodelet 被 manager 加载(轮询 rosservice call <manager>/list)。
wait_nodelet()
{
    local manager="$1"
    local nodelet_name="$2"
    local timeout_s="${3:-60}"
    local start_time=$SECONDS

    log WAIT "等待 nodelet ${nodelet_name}(manager=${manager})..."
    while ! rosservice call "${manager}/list" 2>/dev/null | grep -q "$nodelet_name"; do
        sleep 1
        (( SECONDS - start_time < timeout_s )) || {
            log ERROR "nodelet ${nodelet_name} 未在 ${timeout_s}s 内加载。"
            return 1
        }
    done
    log OK "nodelet ${nodelet_name} 已加载。"
}

# 等待 topic 连续发布 count 条消息。
wait_topic_messages()
{
    local topic="$1"
    local count="${2:-2}"
    local timeout_s="${3:-30}"

    log WAIT "等待 ${topic} 发布 ${count} 条消息..."
    if timeout "$timeout_s" rostopic echo -n "$count" "$topic" >/dev/null 2>&1; then
        log OK "${topic} 正在发布。"
        return 0
    fi
    log ERROR "${topic} 未在 ${timeout_s}s 内发布 ${count} 条消息。"
    return 1
}

# ---------- ROS master / MAVROS 就绪 ----------

# 等待 ROS master(轮询 rostopic list,带超时)。
wait_roscore()
{
    local timeout_s="${1:-60}"
    local start_time=$SECONDS

    log WAIT "等待 ROS master..."
    until rostopic list >/dev/null 2>&1; do
        sleep 1
        (( SECONDS - start_time < timeout_s )) || {
            log ERROR "ROS master 未在 ${timeout_s}s 内就绪。"
            return 1
        }
    done
    log OK "ROS master 就绪。"
}

# 若无 ROS master 则在 gnome-terminal 新 tab 启动 roscore,再等待就绪。
ensure_roscore()
{
    if rostopic list >/dev/null 2>&1; then
        log OK "复用已有 ROS master。"
        return 0
    fi

    gnome-terminal --tab --title="roscore" -- \
        bash -c "roscore; exec bash"

    wait_roscore
}

# 等待 MAVROS 连上飞控(/mavros/state connected:True),回显 mode/armed。
# fcu_info 为可选提示串(如设备路径/波特率),超时时随错误打印。
wait_mavros_connected()
{
    local timeout_s="${1:-120}"
    local fcu_info="${2:-}"
    local start_time=$SECONDS
    local state mode armed

    log WAIT "等待 MAVROS 连接飞控..."
    while (( SECONDS - start_time < timeout_s )); do
        state=$(read_state /mavros/state 2>/dev/null)
        if [ "$(state_value "$state" connected)" = "True" ]; then
            mode=$(state_value "$state" mode)
            armed=$(state_value "$state" armed)
            log OK "MAVROS 已连接: mode=${mode:-unknown}, armed=${armed:-unknown}。"
            return 0
        fi
        sleep 1
    done

    log ERROR "MAVROS 未在 ${timeout_s}s 内连接。${fcu_info:+ $fcu_info}"
    return 1
}

# 关键节点已存在则报错(防止重复启动 MAVROS / manager / controller 等)。
ensure_node_absent()
{
    local node="$1"

    if rosnode list 2>/dev/null | grep -qx "$node"; then
        log ERROR "关键节点已存在: $node"
        return 1
    fi
    return 0
}

# ---------- /mavros/state 纯解析(不带门禁,供脚本自行判断) ----------

# 读一次 state 话题(默认 /mavros/state),stdout 输出原始文本。
read_state()
{
    local topic="${1:-/mavros/state}"
    timeout 3 rostopic echo -n 1 "$topic" 2>/dev/null
}

# 从 state 文本中取某字段值(去掉引号)。
state_value()
{
    local state="$1"
    local key="$2"
    awk -F': ' -v wanted="$key" '$1 == wanted {gsub(/[\047\042]/, "", $2); print $2; exit}' <<<"$state"
}

# ---------- gnome-terminal tab 编排 ----------

# 把一段命令包成临时脚本(source bashrc + 本公共库 + cmd + exec bash),返回脚本路径,
# 供 gnome-terminal -e 使用,规避 -e 的引号嵌套与 $VAR 展开冲突。
# 注意:mk_tab 通常经 $(...) 调用,运行在子 shell,故无法用进程内数组登记;
# cleanup_tabs 改按文件名前缀清理(见下)。
mk_tab()
{
    local f
    f=$(mktemp /tmp/ham_ros_tab.XXXXXX.sh)
    {
        echo '#!/bin/bash'
        echo 'source ~/.bashrc 2>/dev/null'
        echo "source \"$_ROS_COMMON_SELF\""
        printf '%s\n' "$1"
        echo 'exec bash'
    } > "$f"
    chmod +x "$f"
    echo "$f"
}

# 清理 mk_tab 生成的临时 tab 脚本(按固定前缀匹配,供 trap cleanup_tabs EXIT 调用)。
# tab 启动后 gnome-terminal 已把脚本读入 bash,之后删除文件不影响正在运行的 tab。
cleanup_tabs()
{
    rm -f /tmp/ham_ros_tab.*.sh 2>/dev/null
}
