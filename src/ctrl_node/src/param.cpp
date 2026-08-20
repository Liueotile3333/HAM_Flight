#include <cmath>
#include "param.hpp"

namespace ctrl_node
{

    template <typename TName, typename TVal>
    void Parameter_t::read_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if (!nh.getParam(name, val))
        {
            // 只记录不中止, 由下方汇总检查统一拒绝启动
            ROS_FATAL_STREAM("Read param: " << name << " failed.");
            param_read_ok_ = false;
        }
    };

    bool Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
    {
        param_read_ok_ = true;
        read_param(nh, "gain/Kvp0", gain.vel.Kvp0);
        read_param(nh, "gain/Kvp1", gain.vel.Kvp1);
        read_param(nh, "gain/Kvp2", gain.vel.Kvp2);
        read_param(nh, "gain/Kvd0", gain.vel.Kvd0);
        read_param(nh, "gain/Kvd1", gain.vel.Kvd1);
        read_param(nh, "gain/Kvd2", gain.vel.Kvd2);
        read_param(nh, "gain/Kp0", gain.att_pid.Kp0);
        read_param(nh, "gain/Kp1", gain.att_pid.Kp1);
        read_param(nh, "gain/Kp2", gain.att_pid.Kp2);
        read_param(nh, "gain/Ki0", gain.att_pid.Ki0);
        read_param(nh, "gain/Ki1", gain.att_pid.Ki1);
        read_param(nh, "gain/Ki2", gain.att_pid.Ki2);
        read_param(nh, "gain/Kd0", gain.att_pid.Kd0);
        read_param(nh, "gain/Kd1", gain.att_pid.Kd1);
        read_param(nh, "gain/Kd2", gain.att_pid.Kd2);
        read_param(nh, "gain/KAngp0", gain.att_pid.KAngp0);
        read_param(nh, "gain/KAngp1", gain.att_pid.KAngp1);
        read_param(nh, "gain/KAngp2", gain.att_pid.KAngp2);
        read_param(nh, "gain/KAngi0", gain.att_pid.KAngi0);
        read_param(nh, "gain/KAngi1", gain.att_pid.KAngi1);
        read_param(nh, "gain/KAngi2", gain.att_pid.KAngi2);
        read_param(nh, "gain/KAngd0", gain.att_pid.KAngd0);
        read_param(nh, "gain/KAngd1", gain.att_pid.KAngd1);
        read_param(nh, "gain/KAngd2", gain.att_pid.KAngd2);
        read_param(nh, "gain/Trou0", gain.att_ude.Trou0);
        read_param(nh, "gain/Trou1", gain.att_ude.Trou1);
        read_param(nh, "gain/Trou2", gain.att_ude.Trou2);
        read_param(nh, "gain/Tatt0", gain.att_ude.Tatt0);
        read_param(nh, "gain/Tatt1", gain.att_ude.Tatt1);
        read_param(nh, "gain/Tatt2", gain.att_ude.Tatt2);
        read_param(nh, "gain/Trou_min0", gain.att_tvude.Trou_min0);
        read_param(nh, "gain/Trou_min1", gain.att_tvude.Trou_min1);
        read_param(nh, "gain/Trou_min2", gain.att_tvude.Trou_min2);
        read_param(nh, "gain/Trou_max0", gain.att_tvude.Trou_max0);
        read_param(nh, "gain/Trou_max1", gain.att_tvude.Trou_max1);
        read_param(nh, "gain/Trou_max2", gain.att_tvude.Trou_max2);
        read_param(nh, "gain/t_min", gain.att_tvude.t_min);
        read_param(nh, "gain/t_max", gain.att_tvude.t_max);
        read_param(nh, "gain/t_z_min", gain.att_tvude.t_z_min);
        read_param(nh, "gain/t_z_max", gain.att_tvude.t_z_max);
        read_param(nh, "gain/t_xy_min", gain.att_tvude.t_xy_min);
        read_param(nh, "gain/t_xy_max", gain.att_tvude.t_xy_max);
        read_param(nh, "gain/Tatt_min0", gain.att_tvude.Tatt_min0);
        read_param(nh, "gain/Tatt_min1", gain.att_tvude.Tatt_min1);
        read_param(nh, "gain/Tatt_min2", gain.att_tvude.Tatt_min2);
        read_param(nh, "gain/Tatt_max0", gain.att_tvude.Tatt_max0);
        read_param(nh, "gain/Tatt_max1", gain.att_tvude.Tatt_max1);
        read_param(nh, "gain/Tatt_max2", gain.att_tvude.Tatt_max2);
        read_param(nh, "gain/t_att_min", gain.att_tvude.t_att_min);
        read_param(nh, "gain/t_att_max", gain.att_tvude.t_att_max);

        read_param(nh, "takeoff_state/height", takeoff_state.height);
        read_param(nh, "takeoff_state/speed", takeoff_state.speed);

        read_param(nh, "landing_state/speed", landing_state.speed);
        read_param(nh, "landing_state/target_height", landing_state.target_height);
        read_param(nh, "landing_state/dis_arm_height", landing_state.dis_arm_height);

        // 兼容迁移: 优先读新名 frequency, 缺失则回退旧名 frequncy + deprecated warning
        if (!nh.getParam("fsmparam/frequency", fsmparam.frequency))
        {
            if (nh.getParam("fsmparam/frequncy", fsmparam.frequency))
                ROS_WARN("[param] 'fsmparam/frequncy' is deprecated (typo), use 'fsmparam/frequency'.");
            else
            {
                ROS_FATAL("Read param: fsmparam/frequency (and legacy fsmparam/frequncy) failed.");
                return false;
            }
        }

        read_param(nh, "msg_timeout/odom", msg_timeout.odom);
        read_param(nh, "msg_timeout/imu", msg_timeout.imu);
        read_param(nh, "msg_timeout/state", msg_timeout.state);
        read_param(nh, "msg_timeout/extended_state", msg_timeout.extended_state);
        read_param(nh, "msg_timeout/cmd", msg_timeout.cmd);

        read_param(nh, "controller_type", controller_type);
        read_param(nh, "estimator_type", estimator_type);
        read_param(nh, "odom_source", odom_source);
        read_param(nh, "yaw_mode", yaw_mode);
        read_param(nh, "mass", mass);
        read_param(nh, "gra", gra);

        read_param(nh, "kine_cons/vel_ver_max", kine_cons.vel_ver_max);
        read_param(nh, "kine_cons/vel_hor_max", kine_cons.vel_hor_max);
        read_param(nh, "kine_cons/acc_ver_max", kine_cons.acc_ver_max);
        read_param(nh, "kine_cons/acc_hor_max", kine_cons.acc_hor_max);
        read_param(nh, "kine_cons/tilt_max_deg", kine_cons.tilt_max_deg);
        read_param(nh, "kine_cons/omega_roll_max", kine_cons.omega_roll_max);

        read_param(nh, "kine_cons/omega_pitch_max", kine_cons.omega_pitch_max);
        read_param(nh, "kine_cons/omega_yaw_max", kine_cons.omega_yaw_max);

        read_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
        read_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
        read_param(nh, "thrust_model/thr2acc_min", thr_map.thr2acc_min);
        read_param(nh, "thrust_model/thr2acc_max", thr_map.thr2acc_max);

        // 失败汇总检查: 缺参时下方校验会读取未初始化值
        if (!param_read_ok_)
        {
            ROS_FATAL("One or more required params are missing; see FATAL logs above. Refusing to start.");
            return false;
        }

        // ---- 参数合法性校验: 非法值启动即失败,
        //      不让 NaN/Inf 进入 MAVROS setpoint, 也不依赖 NDEBUG 下为空操作的 ROS_BREAK ----
        if (odom_source != 0 && odom_source != 1)
        {
            ROS_FATAL("Invalid odom_source=%d (use 0=world velocity or 1=body velocity).", odom_source);
            return false;
        }
        if (yaw_mode != 0 && yaw_mode != 1)
        {
            ROS_FATAL("Invalid yaw_mode=%d (use 0=fixed heading or 1=trajectory-tangent heading).", yaw_mode);
            return false;
        }
        const auto finite_positive = [](double value)
        {
            return std::isfinite(value) && value > 0.0;
        };

        if (!finite_positive(kine_cons.vel_hor_max) ||
            !finite_positive(kine_cons.vel_ver_max) ||
            !finite_positive(kine_cons.acc_hor_max) ||
            !finite_positive(kine_cons.acc_ver_max) ||
            !finite_positive(kine_cons.tilt_max_deg) ||
            kine_cons.tilt_max_deg >= 60.0)
        {
            ROS_FATAL("Invalid kine_cons safety limits: vel_hor=%.3f vel_ver=%.3f acc_hor=%.3f acc_ver=%.3f tilt=%.3f",
                      kine_cons.vel_hor_max, kine_cons.vel_ver_max,
                      kine_cons.acc_hor_max, kine_cons.acc_ver_max,
                      kine_cons.tilt_max_deg);
            return false;
        }
        if (!(kine_cons.omega_roll_max > 0.0) ||
            !(kine_cons.omega_pitch_max > 0.0) ||
            !(kine_cons.omega_yaw_max > 0.0))
        {
            ROS_FATAL("Invalid body-rate limits: roll=%.3f pitch=%.3f yaw=%.3f rad/s",
                      kine_cons.omega_roll_max,
                      kine_cons.omega_pitch_max,
                      kine_cons.omega_yaw_max);
            return false;
        }
        if (!finite_positive(thr_map.hover_percentage) ||
            thr_map.hover_percentage > 1.0 ||
            !finite_positive(thr_map.thr2acc_min) ||
            !std::isfinite(thr_map.thr2acc_max) ||
            thr_map.thr2acc_max <= thr_map.thr2acc_min)
        {
            ROS_FATAL("Invalid thrust_model safety limits: hover_percentage=%.3f, thr2acc=[%.3f, %.3f]",
                      thr_map.hover_percentage, thr_map.thr2acc_min, thr_map.thr2acc_max);
            return false;
        }

        // 非法值会让 fsm switch 落入 default(不发布)或 if 链全部失配(静默), 须启动即失败
        if (controller_type < 0 || controller_type > 3)
        {
            ROS_FATAL("Invalid controller_type=%d (use 0:position 1:velocity 2:attitude 3:angular_velocity).",
                      controller_type);
            return false;
        }
        if (estimator_type < 0 || estimator_type > 4)
        {
            ROS_FATAL("Invalid estimator_type=%d (use 0:UDE 1:TVUDE 2:PD 3:TTVUDE 4:PID).",
                      estimator_type);
            return false;
        }

        const double controller_gains[] = {
            gain.vel.Kvp0, gain.vel.Kvp1, gain.vel.Kvp2,
            gain.vel.Kvd0, gain.vel.Kvd1, gain.vel.Kvd2,
            gain.att_pid.Kp0, gain.att_pid.Kp1, gain.att_pid.Kp2,
            gain.att_pid.Ki0, gain.att_pid.Ki1, gain.att_pid.Ki2,
            gain.att_pid.Kd0, gain.att_pid.Kd1, gain.att_pid.Kd2,
            gain.att_pid.KAngp0, gain.att_pid.KAngp1, gain.att_pid.KAngp2,
            gain.att_pid.KAngi0, gain.att_pid.KAngi1, gain.att_pid.KAngi2,
            gain.att_pid.KAngd0, gain.att_pid.KAngd1, gain.att_pid.KAngd2};
        for (double gain_value : controller_gains)
        {
            if (!std::isfinite(gain_value))
            {
                ROS_FATAL("Controller gains must all be finite.");
                return false;
            }
        }

        // UDE/TVUDE 时变参数校验 (表驱动, 不随 estimator_type 分支):
        //   * T 类均作分母, 须 finite 且 >0 (0 → NaN 且积分器永久中毒; <0 → 符号
        //     反转正反馈失稳; +Inf 不会被 !(x>0) 拦截, 故显式 isfinite)。
        //   * 调度区间 t_*: span 进入 M_PI/span, 须 min>=0 且 min<max
        //     (min<0 破坏 T_init=T_max 假设; 界倒置会静默互换, 比 NaN 难排查)。
        {
            struct NamedValue
            {
                const char *name;
                double val;
            };
            struct NamedPair
            {
                const char *min_name;
                const char *max_name;
                double min_val;
                double max_val;
            };

            // T 类 (UDE 常值 + TVUDE 上下界): 须 finite 且 > 0
            const NamedValue t_values[] = {
                {"gain/Trou0", gain.att_ude.Trou0}, {"gain/Trou1", gain.att_ude.Trou1}, {"gain/Trou2", gain.att_ude.Trou2},
                {"gain/Tatt0", gain.att_ude.Tatt0}, {"gain/Tatt1", gain.att_ude.Tatt1}, {"gain/Tatt2", gain.att_ude.Tatt2},
                {"gain/Trou_min0", gain.att_tvude.Trou_min0}, {"gain/Trou_min1", gain.att_tvude.Trou_min1}, {"gain/Trou_min2", gain.att_tvude.Trou_min2},
                {"gain/Trou_max0", gain.att_tvude.Trou_max0}, {"gain/Trou_max1", gain.att_tvude.Trou_max1}, {"gain/Trou_max2", gain.att_tvude.Trou_max2},
                {"gain/Tatt_min0", gain.att_tvude.Tatt_min0}, {"gain/Tatt_min1", gain.att_tvude.Tatt_min1}, {"gain/Tatt_min2", gain.att_tvude.Tatt_min2},
                {"gain/Tatt_max0", gain.att_tvude.Tatt_max0}, {"gain/Tatt_max1", gain.att_tvude.Tatt_max1}, {"gain/Tatt_max2", gain.att_tvude.Tatt_max2}};

            // 调度区间 (四组各自独立): 双双 finite, min>=0 且 min<max
            const NamedPair sched_pairs[] = {
                {"gain/t_min", "gain/t_max", gain.att_tvude.t_min, gain.att_tvude.t_max},
                {"gain/t_z_min", "gain/t_z_max", gain.att_tvude.t_z_min, gain.att_tvude.t_z_max},
                {"gain/t_xy_min", "gain/t_xy_max", gain.att_tvude.t_xy_min, gain.att_tvude.t_xy_max},
                {"gain/t_att_min", "gain/t_att_max", gain.att_tvude.t_att_min, gain.att_tvude.t_att_max}};

            // T 上下界: min < max
            const NamedPair t_pairs[] = {
                {"gain/Trou_min0", "gain/Trou_max0", gain.att_tvude.Trou_min0, gain.att_tvude.Trou_max0},
                {"gain/Trou_min1", "gain/Trou_max1", gain.att_tvude.Trou_min1, gain.att_tvude.Trou_max1},
                {"gain/Trou_min2", "gain/Trou_max2", gain.att_tvude.Trou_min2, gain.att_tvude.Trou_max2},
                {"gain/Tatt_min0", "gain/Tatt_max0", gain.att_tvude.Tatt_min0, gain.att_tvude.Tatt_max0},
                {"gain/Tatt_min1", "gain/Tatt_max1", gain.att_tvude.Tatt_min1, gain.att_tvude.Tatt_max1},
                {"gain/Tatt_min2", "gain/Tatt_max2", gain.att_tvude.Tatt_min2, gain.att_tvude.Tatt_max2}};

            for (const NamedValue &nv : t_values)
                if (!(std::isfinite(nv.val) && nv.val > 0.0))
                {
                    ROS_FATAL("Invalid UDE/TVUDE param %s=%.6f (must be finite and > 0).",
                              nv.name, nv.val);
                    return false;
                }

            for (const NamedPair &p : sched_pairs)
                if (!(std::isfinite(p.min_val) && std::isfinite(p.max_val) &&
                      p.min_val >= 0.0 && p.min_val < p.max_val))
                {
                    ROS_FATAL("Invalid TVUDE schedule %s=%.6f, %s=%.6f (both finite, min>=0 and min<max required).",
                              p.min_name, p.min_val, p.max_name, p.max_val);
                    return false;
                }

            for (const NamedPair &p : t_pairs)
                if (!(p.min_val < p.max_val))
                {
                    ROS_FATAL("Invalid TVUDE bounds %s=%.6f must be < %s=%.6f.",
                              p.min_name, p.min_val, p.max_name, p.max_val);
                    return false;
                }
        }
        // !(x>0) 同时拦截 <=0 与 NaN: mass/gra<=0 → 推力映射除零/失真;
        // frequency<=0 → 定时器除零; msg_timeout<=0 → is_received 恒 false。
        if (!finite_positive(mass) || !finite_positive(gra))
        {
            ROS_FATAL("Invalid physical params: mass=%.4f kg, gra=%.4f m/s^2 (must be > 0).", mass, gra);
            return false;
        }
        if (!(fsmparam.frequency > 0))
        {
            ROS_FATAL("Invalid fsmparam/frequency=%d (must be > 0).", fsmparam.frequency);
            return false;
        }
        if (!finite_positive(msg_timeout.odom) ||
            !finite_positive(msg_timeout.imu) ||
            !finite_positive(msg_timeout.state) ||
            !finite_positive(msg_timeout.extended_state) ||
            !finite_positive(msg_timeout.cmd))
        {
            ROS_FATAL("Invalid msg_timeout: odom=%.3f imu=%.3f state=%.3f extended_state=%.3f cmd=%.3f "
                      "(must be finite and > 0).",
                      msg_timeout.odom, msg_timeout.imu,
                      msg_timeout.state, msg_timeout.extended_state,
                      msg_timeout.cmd);
            return false;
        }
        if (!finite_positive(takeoff_state.height) ||
            !finite_positive(takeoff_state.speed))
        {
            ROS_FATAL("Invalid takeoff parameters: height=%.3f speed=%.3f (must be finite and > 0).",
                      takeoff_state.height, takeoff_state.speed);
            return false;
        }
        // 旧版平滑下降配置仍做静态检查，便于保持配置文件向后兼容；
        // 当前 LAND 直接请求 PX4 AUTO.LAND，不再使用世界系 target_height 作为交接门限。
        if (!finite_positive(landing_state.speed))
        {
            ROS_FATAL("Invalid landing_state/speed=%.4f m/s (must be > 0).", landing_state.speed);
            return false;
        }
        if (!finite_positive(landing_state.dis_arm_height))
        {
            ROS_FATAL("Invalid landing_state/dis_arm_height=%.4f m (must be > 0).", landing_state.dis_arm_height);
            return false;
        }
        if (!std::isfinite(landing_state.target_height))
        {
            ROS_FATAL("Invalid landing_state/target_height=%.4f (must be finite).", landing_state.target_height);
            return false;
        }

        return true;
    }
}
