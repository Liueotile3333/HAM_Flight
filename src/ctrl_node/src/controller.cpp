#include "controller.hpp"

#include <algorithm>

namespace Controller
{

    // 时变 UDE 参数求值: T(t) 过渡区 (t_min, t_max] 内余弦由 Tmax 平滑到 Tmin,
    // 区外取端值; dInvT_dt = d(1/T)/dt, 区外为 0 (旧实现符号反, 已修正为 +)。
    // T_init = T(0) = Tmax (t_min≥0), 直接用上界, 无需每周期求值。
    struct TVaryingT
    {
        double T;
        double dInvT_dt;
    };

    struct TVaryingT3
    {
        Eigen::Vector3d T;
        Eigen::Vector3d dInvT_dt;
    };

    static TVaryingT evalTVarying(double t, double t_min, double t_max,
                                  double Tmin, double Tmax)
    {
        if (t <= t_min)
            return {Tmax, 0.0};
        if (t > t_max)
            return {Tmin, 0.0};

        const double half_dT = (Tmax - Tmin) / 2.0;
        const double mid = (Tmax + Tmin) / 2.0;
        const double span = t_max - t_min;
        const double theta = M_PI * (t - t_min) / span;
        const double T = half_dT * std::cos(theta) + mid;
        // dT/dt = -half_dT·sinθ·(π/span);  d(1/T)/dt = -(dT/dt)/T² = +half_dT·sinθ·(π/span)/T²
        const double dInvT_dt = half_dT * std::sin(theta) * (M_PI / span) / (T * T);
        return {T, dInvT_dt};
    }

    static TVaryingT3 evalTVarying(double t, double t_min, double t_max,
                                   const Eigen::Vector3d &Tmin, const Eigen::Vector3d &Tmax)
    {
        if (t <= t_min)
            return {Tmax, Eigen::Vector3d::Zero()};
        if (t > t_max)
            return {Tmin, Eigen::Vector3d::Zero()};

        TVaryingT3 r;
        const double span = t_max - t_min;
        const double theta = M_PI * (t - t_min) / span;
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        const double pi_over_span = M_PI / span;
        for (int i = 0; i < 3; ++i)
        {
            const double half_dT = (Tmax(i) - Tmin(i)) / 2.0;
            const double mid = (Tmax(i) + Tmin(i)) / 2.0;
            const double T = half_dT * c + mid;
            r.T(i) = T;
            r.dInvT_dt(i) = half_dT * s * pi_over_span / (T * T);
        }
        return r;
    }

    void Position_Control::init(ctrl_node::Parameter_t &param)
    {
        param_ = param;
    }

    quadrotor_msgs::Px4ctrlDebug Position_Control::calculateControl(const Desired_State_t &des,
                                                                    const ctrl_node::Odom_Data_t &odom,
                                                                    Controller_Output_t &u)
    {
        // 位置直发: 期望位置/偏航直接下发 PX4 内部位置环, 本节点不做位置反馈
        (void)odom;
        quadrotor_msgs::Px4ctrlDebug data;
        u.position = des.p;
        u.yaw = des.yaw;
        return data;
    }

    void Velocity_Control::init(ctrl_node::Parameter_t &param)
    {
        param_ = param;
        input.vel_last.setZero();
        input.yaw_last = 0;
        last_time_ = ros::Time(0); // 首拍标记: isZero()=true → 首个控制周期跳过速率限幅
    }

    quadrotor_msgs::Px4ctrlDebug Velocity_Control::calculateControl(const Desired_State_t &des,
                                                                    const ctrl_node::Odom_Data_t &odom,
                                                                    Controller_Output_t &u,
                                                                    const ros::Time &now)
    {
        Eigen::Vector3d Kp(param_.gain.vel.Kvp0, param_.gain.vel.Kvp1, param_.gain.vel.Kvp2);
        Eigen::Vector3d Kd(param_.gain.vel.Kvd0, param_.gain.vel.Kvd1, param_.gain.vel.Kvd2);

        Eigen::Vector3d vel_max(param_.kine_cons.vel_hor_max, param_.kine_cons.vel_hor_max, param_.kine_cons.vel_ver_max);
        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max, param_.kine_cons.acc_hor_max, param_.kine_cons.acc_ver_max);

        // 真实控制周期 dt 防跳变(门限同 getSafeControlDt): 首拍/sim 复位等跳变时
        // dt_valid=false, 本周期跳过速率限幅, 限幅基准照常更新
        double dt = 0.0;
        bool dt_valid = false;
        if (!last_time_.isZero() && now.isValid() && last_time_.isValid())
        {
            dt = (now - last_time_).toSec();
            constexpr double kMaxControlDt = 1.0; // [s]
            if (std::isfinite(dt) && dt > 0.0 && dt <= kMaxControlDt)
                dt_valid = true;
        }
        last_time_ = now;

        // PD: P=位置误差, D=速度误差(替代旧的误差差分, 无差分噪声、不随频率漂移;
        // Kvd 变为速度误差增益, 启用 velocity 模式前需重新整定)
        Eigen::Vector3d pos_err(des.p - odom.p);
        Eigen::Vector3d vel_err(des.v - odom.v);
        u.velocity = Kp.asDiagonal() * pos_err + Kd.asDiagonal() * vel_err;

        // limit vel
        u.velocity = u.velocity.cwiseMax(-vel_max).cwiseMin(vel_max);

        // limit acc: acc_max×dt = 单周期速度增量上限(旧实现量纲错配, 等效上限随频率漂移)
        if (dt_valid)
        {
            Eigen::Vector3d dv_max = acc_max * dt;
            Eigen::Vector3d dv(u.velocity - input.vel_last);
            dv = dv.cwiseMax(-dv_max).cwiseMin(dv_max);
            u.velocity = input.vel_last + dv;
        }
        input.vel_last = u.velocity;

        // limit yaw rate: omega_yaw_max[rad/s] × dt[s] = 单周期 yaw 增量上限 dψ_max[rad]。
        double yaw_err(des.yaw - input.yaw_last);
        yaw_err = std::remainder(yaw_err, 2 * M_PI); // ±π 回绕修正(避免 des.yaw 与 yaw_last 跨边界时误差突变为 ~2π)
        if (dt_valid)
        {
            const double dyaw_max = param_.kine_cons.omega_yaw_max * dt;
            yaw_err = yaw_err > dyaw_max ? dyaw_max : yaw_err;
            yaw_err = yaw_err < -dyaw_max ? -dyaw_max : yaw_err;
        }
        u.yaw = input.yaw_last + yaw_err;
        input.yaw_last = u.yaw;

        // debug
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);

        debug_msg_.des_v_x = pos_err(0);
        debug_msg_.des_v_y = pos_err(1);
        debug_msg_.des_v_z = pos_err(2);

        debug_msg_.cmd_v_x = u.velocity(0);
        debug_msg_.cmd_v_y = u.velocity(1);
        debug_msg_.cmd_v_z = u.velocity(2);

        debug_msg_.des_yaw = u.yaw;

        return debug_msg_;
    }

    // Attitude_Angular_Control: 整套控制核心。
    //   位置环/姿态环主控(calculateControl / calculateControlCMD)、加速度安全限幅、
    //   roll/pitch 反解、yaw 解算、推力映射; 时变 UDE 参数由文件级 evalTVarying 提供。
    bool Attitude_Angular_Control::init(ctrl_node::Parameter_t &param, const ros::NodeHandle &nh)
    {
        param_ = param;
        resetThrustMapping();

        // 增益一次性缓存, 避免每周期从 param_.gain.* 重建向量
        Kp_ << param_.gain.att_pid.Kp0, param_.gain.att_pid.Kp1, param_.gain.att_pid.Kp2;
        Ki_ << param_.gain.att_pid.Ki0, param_.gain.att_pid.Ki1, param_.gain.att_pid.Ki2;
        Kd_ << param_.gain.att_pid.Kd0, param_.gain.att_pid.Kd1, param_.gain.att_pid.Kd2;
        Trou_ << param_.gain.att_ude.Trou0, param_.gain.att_ude.Trou1, param_.gain.att_ude.Trou2;
        Trou_min_ << param_.gain.att_tvude.Trou_min0, param_.gain.att_tvude.Trou_min1, param_.gain.att_tvude.Trou_min2;
        Trou_max_ << param_.gain.att_tvude.Trou_max0, param_.gain.att_tvude.Trou_max1, param_.gain.att_tvude.Trou_max2;
        t_min_ = param_.gain.att_tvude.t_min;
        t_max_ = param_.gain.att_tvude.t_max;
        t_z_min_ = param_.gain.att_tvude.t_z_min;
        t_z_max_ = param_.gain.att_tvude.t_z_max;
        t_xy_min_ = param_.gain.att_tvude.t_xy_min;
        t_xy_max_ = param_.gain.att_tvude.t_xy_max;
        KAngp_ << param_.gain.att_pid.KAngp0, param_.gain.att_pid.KAngp1, param_.gain.att_pid.KAngp2;
        KAngi_ << param_.gain.att_pid.KAngi0, param_.gain.att_pid.KAngi1, param_.gain.att_pid.KAngi2;
        KAngd_ << param_.gain.att_pid.KAngd0, param_.gain.att_pid.KAngd1, param_.gain.att_pid.KAngd2;
        t_att_min_ = param_.gain.att_tvude.t_att_min;
        t_att_max_ = param_.gain.att_tvude.t_att_max;
        Tatt_ << param_.gain.att_ude.Tatt0, param_.gain.att_ude.Tatt1, param_.gain.att_ude.Tatt2;
        Tatt_min_ << param_.gain.att_tvude.Tatt_min0, param_.gain.att_tvude.Tatt_min1, param_.gain.att_tvude.Tatt_min2;
        Tatt_max_ << param_.gain.att_tvude.Tatt_max0, param_.gain.att_tvude.Tatt_max1, param_.gain.att_tvude.Tatt_max2;

        last_eul_err.setZero();

        // 保存节点句柄并初始化订阅变量
        nh_ = nh;
        time_diff_ = 0.0;
        time_future_ = 0.0;
        future_velocity_.setZero();
        initial_velocity_.setZero();
        // 初始化航向角相关变量
        last_yaw_ = 0.0;
        is_first_in_calculate_yaw = true;

        // 仿真里程计订阅
        time_diff_sub_ = nh_.subscribe<std_msgs::Float64>("/drone0/sim_odom/time_diff", 1,
                                                          &Attitude_Angular_Control::timeDiffCallback, this);
        time_future_sub_ = nh_.subscribe<std_msgs::Float64>("/drone0/sim_odom/time_future", 1,
                                                            &Attitude_Angular_Control::timefutureCallback, this);
        future_velocity_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/future_velocity", 1,
                                                                          &Attitude_Angular_Control::futurevelocityCallback, this);
        initial_velocity_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/initial_velocity", 1,
                                                                           &Attitude_Angular_Control::initialvelocityCallback, this);

        // 气动模块初始化, 失败须向上传播 (TABLE 数据库错误不允许继续运行)
        if (!aero_model_.init(param_, nh_))
        {
            ROS_FATAL(
                "[CTRL]: aerodynamic model initialization failed.");

            return false;
        }

        return true;
    }

    void Attitude_Angular_Control::timeDiffCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        time_diff_ = msg->data;
    }
    void Attitude_Angular_Control::timefutureCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        time_future_ = msg->data;
    }
    void Attitude_Angular_Control::futurevelocityCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        future_velocity_ << msg->point.x, msg->point.y, msg->point.z;
    }
    void Attitude_Angular_Control::initialvelocityCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        initial_velocity_ << msg->point.x, msg->point.y, msg->point.z;
    }

    double Attitude_Angular_Control::getTimeDiff() const { return time_diff_; }
    double Attitude_Angular_Control::getTimefuture() const { return time_future_; }
    Eigen::Vector3d Attitude_Angular_Control::getfuturevelocity() const { return future_velocity_; }
    Eigen::Vector3d Attitude_Angular_Control::getinitialvelocity() const { return initial_velocity_; }

    // 加速度安全限幅 + 气动前馈:
    //   非有限保护 → 垂直/水平限幅(水平按模长保方向) → 重力+气动补偿
    //   → 最小垂直比力 → 倾斜角硬约束
    Eigen::Vector3d Attitude_Angular_Control::makeSafeTotalAcceleration(const Eigen::Vector3d &acc_cmd, const ctrl_node::Odom_Data_t &odom) const
    {
        Eigen::Vector3d limited = acc_cmd;

        if (!limited.allFinite())
        {
            ROS_ERROR_THROTTLE(1.0,
                               "[CTRL]: non-finite translational acceleration command; replacing with zero correction.");
            limited.setZero();
        }

        // Vertical bound is applied to the translational correction before gravity.
        limited.z() = std::max(-param_.kine_cons.acc_ver_max,
                               std::min(param_.kine_cons.acc_ver_max, limited.z()));

        // Limit the TOTAL horizontal command by vector norm. This preserves the
        // commanded XY direction; component-wise clipping would distort it.
        const double a_xy_norm = limited.head<2>().norm();
        if (a_xy_norm > param_.kine_cons.acc_hor_max && a_xy_norm > 1e-9)
        {
            const double scale = param_.kine_cons.acc_hor_max / a_xy_norm;
            limited.x() *= scale;
            limited.y() *= scale;
            ROS_WARN_THROTTLE(1.0,
                              "[CTRL]: horizontal acceleration saturated: %.3f -> %.3f m/s^2",
                              a_xy_norm, param_.kine_cons.acc_hor_max);
        }

        // 转换为总比力: 重力补偿 + 气动前馈(升力翼)。
        // AeroModel 解算世界系比力补偿 compensation_acc_world = -F_aero_world/m, 直接相加;
        // 悬停/低速/风场失效时为 0, 退化为标准重力补偿。详见 AeroModel::computeFeedforward。
        const AeroModel::AeroFeedforward aero = aero_model_.computeFeedforward(odom);

        limited += aero.compensation_acc_world;
        limited.z() += param_.gra;

        // Keep a positive vertical component so attitude/thrust inversion remains
        // well defined even if an estimator momentarily produces a bad z command.
        const double min_total_z = 0.10 * param_.gra;
        if (limited.z() < min_total_z)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[CTRL]: total vertical acceleration too small (%.3f); clamped to %.3f m/s^2",
                              limited.z(), min_total_z);
            limited.z() = min_total_z;
        }

        // Secondary hard tilt constraint. With the current acc_hor_max this is
        // normally inactive, but it prevents future gain changes from producing
        // the 70-100 deg commands observed in the failed rosbag.
        const double tilt_max = param_.kine_cons.tilt_max_deg * M_PI / 180.0;
        const double tilt_xy_max = limited.z() * std::tan(tilt_max);
        const double total_xy_norm = limited.head<2>().norm();
        if (total_xy_norm > tilt_xy_max && total_xy_norm > 1e-9)
        {
            const double scale = tilt_xy_max / total_xy_norm;
            limited.x() *= scale;
            limited.y() *= scale;
            ROS_WARN_THROTTLE(1.0,
                              "[CTRL]: desired tilt saturated at %.1f deg",
                              param_.kine_cons.tilt_max_deg);
        }

        return limited;
    }

    // 期望总加速度 → roll/pitch: ZYX 精确反解, 小角度退化为原始公式, 用期望偏航
    void Attitude_Angular_Control::accelerationToRollPitch(const Eigen::Vector3d &total_acc,
                                                           double yaw,
                                                           double &roll,
                                                           double &pitch) const
    {
        const double thrust_norm = total_acc.norm();
        if (!std::isfinite(thrust_norm) || thrust_norm < 1e-6)
        {
            roll = 0.0;
            pitch = 0.0;
            ROS_ERROR_THROTTLE(1.0,
                               "[CTRL]: invalid total acceleration while generating desired attitude.");
            return;
        }

        const double sy = std::sin(yaw);
        const double cy = std::cos(yaw);

        // Exact inversion for a ZYX attitude parameterization. It reduces to the
        // original small-angle equations near hover, but remains well behaved at
        // finite tilt and uses desired yaw rather than current yaw.
        const double a_roll = total_acc.x() * sy - total_acc.y() * cy;
        const double a_pitch = total_acc.x() * cy + total_acc.y() * sy;
        const double sin_roll = std::max(-1.0, std::min(1.0, a_roll / thrust_norm));

        roll = std::asin(sin_roll);
        pitch = std::atan2(a_pitch, total_acc.z());

        const double tilt_max = param_.kine_cons.tilt_max_deg * M_PI / 180.0;
        roll = std::max(-tilt_max, std::min(tilt_max, roll));
        pitch = std::max(-tilt_max, std::min(tilt_max, pitch));
    }

    // 位置环共用核心: PD 前馈 + 扰动估计(UDE/TVUDE/TTVUDE/PID) → 安全限幅后的
    // 期望总加速度(含 g); d_acc / T_pose / Tdot_pose 供 debug。
    // pid_integral_always: PID 是否非轨迹段也积分 —— calculateControl 传 true
    // (HOVER+MISSION 均积分消稳态误差), calculateControlCMD 传 false (仅 trigger 段)。
    Eigen::Vector3d Attitude_Angular_Control::computePositionLoopAccel(
        const Desired_State_t &des,
        const ctrl_node::Odom_Data_t &odom,
        const ctrl_node::Mission_Trigger_t &mission,
        const ros::Time &t,
        bool pid_integral_always,
        Eigen::Vector3d &d_acc,
        Eigen::Vector3d &T_pose,
        Eigen::Vector3d &Tdot_pose)
    {
        if (is_first_in_control)
        {
            last_time = t;
        }

        const Eigen::Vector3d &Trou = Trou_; // Pose-UDE 参数
        const double &t_min = t_min_;
        const double &t_max = t_max_;
        const double &t_z_min = t_z_min_;
        const double &t_z_max = t_z_max_;
        const double &t_xy_min = t_xy_min_;
        const double &t_xy_max = t_xy_max_;
        const Eigen::Vector3d &Trou_min = Trou_min_;
        const Eigen::Vector3d &Trou_max = Trou_max_;
        const Eigen::Vector3d &Kp = Kp_;
        const Eigen::Vector3d &Ki = Ki_;
        const Eigen::Vector3d &Kd = Kd_;

        Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
        T_pose.setZero();
        Tdot_pose.setZero();
        d_acc.setZero();

        des_acc = des.a + Kd.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

        // 0:UDE 1:TVUDE 2:PD 3:TTVUDE 4:PID
        if (param_.estimator_type == 1) // 1:TVUDE
        {
            if (mission.active)
            {
                if (is_first_in_pubtrigger)
                {
                    init_TTV_t = t;
                    init_taj_state = odom.v;
                    is_first_in_pubtrigger = false;
                }
                if (is_first_in_control)
                { // 累积误差清零
                    u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_control = false;
                }
                else
                {
                    double dt = 0.0;
                    if (!getSafeControlDt(t, last_time, dt))
                    {
                        u0_integral_pos.setZero();
                        ROS_WARN_THROTTLE(1.0, "[CTRL]: invalid TVUDE position dt; estimator reset for this cycle.");
                    }
                    else
                    {
                        const double t_rel = (t - init_TTV_t).toSec();
                        const TVaryingT3 tv = evalTVarying(t_rel, t_min, t_max, Trou_min, Trou_max);
                        d_acc(0) = odom.v(0) / tv.T(0) - init_taj_state(0) / Trou_max(0);
                        d_acc(1) = odom.v(1) / tv.T(1) - init_taj_state(1) / Trou_max(1);
                        d_acc(2) = odom.v(2) / tv.T(2) - init_taj_state(2) / Trou_max(2);
                        u0_integral_pos(0) += (odom.v(0) * tv.dInvT_dt(0) + des_acc(0) / tv.T(0)) * dt;
                        u0_integral_pos(1) += (odom.v(1) * tv.dInvT_dt(1) + des_acc(1) / tv.T(1)) * dt;
                        u0_integral_pos(2) += (odom.v(2) * tv.dInvT_dt(2) + des_acc(2) / tv.T(2)) * dt;
                        d_acc -= u0_integral_pos;
                        T_pose = tv.T;
                        Tdot_pose = tv.dInvT_dt;
                    }
                }
            }
        }
        else if (param_.estimator_type == 0) // 0:UDE
        {
            if (mission.active)
            {
                if (is_first_in_pubtrigger)
                {
                    init_taj_state = odom.v;
                    is_first_in_pubtrigger = false;
                }
                if (is_first_in_control)
                { // 累积误差清零
                    u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_control = false;
                }
                else
                {
                    double dt = 0.0;
                    if (!getSafeControlDt(t, last_time, dt))
                    {
                        u0_integral_pos.setZero();
                        ROS_WARN_THROTTLE(1.0, "[CTRL]: invalid UDE position dt; estimator reset for this cycle.");
                    }
                    else
                    {
                        u0_integral_pos += des_acc * dt;
                        d_acc = (odom.v - init_taj_state - u0_integral_pos)
                                    .cwiseQuotient(Trou);
                        T_pose = Trou;
                    }
                }
            }
        }
        // 2:PD —— 无扰动估计, d_acc 保持 0(默认), last_time 由末尾统一更新。
        else if (param_.estimator_type == 3) // 3:TTVUDE
        {
            if (mission.active)
            {
                if (is_first_in_pubtrigger)
                {
                    init_TTV_t = t;
                    init_taj_state = odom.v;
                    is_first_in_pubtrigger = false;
                }
                if (is_first_in_control)
                { // 累积误差清零
                    u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_control = false;
                }
                else
                {
                    double dt = 0.0;
                    if (!getSafeControlDt(t, last_time, dt))
                    {
                        u0_integral_pos.setZero();
                        ROS_WARN_THROTTLE(1.0, "[CTRL]: invalid TTVUDE position dt; estimator reset for this cycle.");
                    }
                    else
                    {
                        const double t_rel = (t - init_TTV_t).toSec();
                        const TVaryingT3 tv_xy = evalTVarying(t_rel, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        const TVaryingT tv_z = evalTVarying(t_rel, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        d_acc(0) = odom.v(0) / tv_xy.T(0) - init_taj_state(0) / Trou_max(0);
                        d_acc(1) = odom.v(1) / tv_xy.T(1) - init_taj_state(1) / Trou_max(1);
                        d_acc(2) = odom.v(2) / tv_z.T - init_taj_state(2) / Trou_max(2);
                        u0_integral_pos(0) += (odom.v(0) * tv_xy.dInvT_dt(0) + des_acc(0) / tv_xy.T(0)) * dt;
                        u0_integral_pos(1) += (odom.v(1) * tv_xy.dInvT_dt(1) + des_acc(1) / tv_xy.T(1)) * dt;
                        u0_integral_pos(2) += (odom.v(2) * tv_z.dInvT_dt + des_acc(2) / tv_z.T) * dt;
                        d_acc -= u0_integral_pos;
                        T_pose << tv_xy.T(0), tv_xy.T(1), tv_z.T;
                        Tdot_pose << tv_xy.dInvT_dt(0), tv_xy.dInvT_dt(1), tv_z.dInvT_dt;
                    }
                }
            }
        }
        else if (param_.estimator_type == 4) // 4:PID 抗饱和
        {
            // 记录轨迹起点速度(仅 MISSION 触发首次)。
            if (mission.active && is_first_in_pubtrigger)
            {
                init_taj_state = odom.v;
                is_first_in_pubtrigger = false;
            }
            // 位置环积分: pid_integral_always=true 时 HOVER+MISSION 均积分(消除
            // 推力映射/气动/CG 偏置稳态误差); false 时仅 trigger 段积分
            if (pid_integral_always || mission.active)
            {
                if (is_first_in_control)
                {
                    pos_err_integral_ = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_control = false;
                }
                else
                {
                    double dt = 0.0;
                    const bool dt_valid = getSafeControlDt(t, last_time, dt);
                    const Eigen::Vector3d pos_err = des.p - odom.p;

                    // I 项贡献上限沿用加速度包络, 后续可加专用限幅参数
                    const Eigen::Vector3d i_acc_max(
                        param_.kine_cons.acc_hor_max,
                        param_.kine_cons.acc_hor_max,
                        param_.kine_cons.acc_ver_max);

                    if (dt_valid)
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            const double i_output = Ki(i) * pos_err_integral_(i);
                            const bool upper_saturated = i_output >= i_acc_max(i) - 1e-9;
                            const bool lower_saturated = i_output <= -i_acc_max(i) + 1e-9;
                            const bool drives_upper = upper_saturated && pos_err(i) > 0.0;
                            const bool drives_lower = lower_saturated && pos_err(i) < 0.0;

                            // Conditional integration: 饱和后只允许有助于退出饱和的积分
                            if (!drives_upper && !drives_lower)
                            {
                                pos_err_integral_(i) += pos_err(i) * dt;
                            }
                            // 积分状态硬上界
                            if (std::abs(Ki(i)) > 1e-9)
                            {
                                const double integral_max = i_acc_max(i) / std::abs(Ki(i));
                                pos_err_integral_(i) =
                                    std::max(
                                        -integral_max,
                                        std::min(
                                            integral_max,
                                            pos_err_integral_(i)));
                            }
                        }
                    }

                    const Eigen::Vector3d i_acc = Ki.asDiagonal() * pos_err_integral_;
                    d_acc = -i_acc; // 保持符号关系: 后面 des_acc -= d_acc
                }
            }
        }

        last_time = t; // 统一更新
        des_acc -= d_acc;
        des_acc = makeSafeTotalAcceleration(des_acc, odom);
        return des_acc;
    }

    // 填充两个主控共用的 debug 字段; des_q 由调用方传入, 特有字段各自补
    void Attitude_Angular_Control::fillCommonDebug(const Desired_State_t &des,
                                                   const ctrl_node::Odom_Data_t &odom,
                                                   const Eigen::Vector3d &des_acc,
                                                   const Eigen::Quaterniond &des_q,
                                                   double des_yaw,
                                                   double thrust)
    {
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);

        debug_msg_.des_v_x = des.v(0);
        debug_msg_.des_v_y = des.v(1);
        debug_msg_.des_v_z = des.v(2);

        debug_msg_.odom_v_x = odom.v(0);
        debug_msg_.odom_v_y = odom.v(1);
        debug_msg_.odom_v_z = odom.v(2);

        debug_msg_.odom_p_x = odom.p(0);
        debug_msg_.odom_p_y = odom.p(1);
        debug_msg_.odom_p_z = odom.p(2);

        debug_msg_.des_a_x = des_acc(0);
        debug_msg_.des_a_y = des_acc(1);
        debug_msg_.des_a_z = des_acc(2);

        debug_msg_.des_q_x = des_q.x();
        debug_msg_.des_q_y = des_q.y();
        debug_msg_.des_q_z = des_q.z();
        debug_msg_.des_q_w = des_q.w();

        debug_msg_.des_yaw = des_yaw;

        debug_msg_.des_thr = thrust;
    }

    // 推力采样(仅 accurate_thrust_model), 滑动窗口 100, 供在线辨识 thr2acc
    void Attitude_Angular_Control::recordThrustSample(double thrust)
    {
        if (param_.thr_map.accurate_thrust_model)
        {
            timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust));
            while (timed_thrust_.size() > 100)
            {
                timed_thrust_.pop();
            }
        }
    }

    // 统一 dt 防跳变查询(只读, last 由调用方用完后自行更新);
    // false 时 dt=0, 调用方应跳过本周期积分。
    // 拒绝: 未初始化/时间非法/非有限/非正/过大(sim 复位等跳变, 防积分 wind-up)
    bool Attitude_Angular_Control::getSafeControlDt(const ros::Time &now,
                                                    const ros::Time &last,
                                                    double &dt) const
    {
        // last 默认构造为 0 → 首个控制周期; 时间非法 → sim 重启等情况。
        if (last.isZero() || !now.isValid() || !last.isValid())
        {
            dt = 0.0;
            return false;
        }

        dt = (now - last).toSec();

        // 最多接受 5 个标称控制周期；最低 50 ms 兼容低频调试配置。
        const double nominal_dt = 1.0 / static_cast<double>(param_.fsmparam.frequency);
        const double kMaxControlDt = std::max(0.05, 5.0 * nominal_dt);
        if (!std::isfinite(dt) || dt <= 0.0 || dt > kMaxControlDt)
        {
            dt = 0.0;
            return false;
        }

        return true;
    }

    void Attitude_Angular_Control::handleMissionTrigger(
        const ctrl_node::Mission_Trigger_t &mission,
        const ctrl_node::Odom_Data_t &odom,
        const ros::Time &now)
    {
        if (mission.sequence == 0U || mission.sequence == last_mission_sequence_)
        {
            return;
        }

        last_mission_sequence_ = mission.sequence;
        is_first_in_control = true;
        is_first_in_att_control = true;
        is_first_in_pubtrigger = true;
        is_first_in_att_pubtrigger = true;
        u0_integral_pos.setZero();
        u0_integral_att.setZero();
        pos_err_integral_.setZero();
        eul_integral_.setZero();
        init_taj_state = odom.v;
        init_att_euler_ = q2euler(odom.q);
        last_eul_err.setZero();
        last_time = now;
        last_att_time = now;
        is_first_in_calculate_yaw = true;
        yaw_rate_inited_ = false;

        ROS_INFO("[CTRL]: mission sequence %llu accepted; estimator state reset.",
                 static_cast<unsigned long long>(mission.sequence));
    }

    // 位置环主控: 扰动估计 → 期望加速度 → 安全限幅 → 推力/姿态输出
    quadrotor_msgs::Px4ctrlDebug Attitude_Angular_Control::calculateControl(const Desired_State_t &des,
                                                                            const ctrl_node::Mission_Trigger_t &mission,
                                                                            const ctrl_node::Odom_Data_t &odom,
                                                                            const ctrl_node::Imu_Data_t &imu,
                                                                            Controller_Output_t &u,
                                                                            ros::Time &t)
    {
        handleMissionTrigger(mission, odom, t);

        // 位置环: PD 前馈 + 扰动估计 → 安全限幅后的期望总加速度(含 g)
        Eigen::Vector3d d_acc(0.0, 0.0, 0.0);
        Eigen::Vector3d T_pose(0.0, 0.0, 0.0);
        Eigen::Vector3d Tdot_pose(0.0, 0.0, 0.0);
        Eigen::Vector3d des_acc = computePositionLoopAccel(des, odom, mission, t,
                                                           /*pid_integral_always=*/true,
                                                           d_acc, T_pose, Tdot_pose);
        (void)d_acc; // 单环模式不输出扰动估计 / TVUDE 参数
        (void)T_pose;
        (void)Tdot_pose;

        // 期望偏航直接取 des.yaw(HOVER 固定航向 / MISSION 规划器给定),
        // 避免 future_velocity 推算在悬停时与 roll/pitch 耦合振荡
        double des_trajectory_yaw = des.yaw;

        double roll = 0.0;
        double pitch = 0.0;
        accelerationToRollPitch(des_acc, des_trajectory_yaw, roll, pitch);
        u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

        // ZYX composition: yaw -> pitch -> roll.
        Eigen::Quaterniond q = Eigen::AngleAxisd(des_trajectory_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        u.q = imu.q * odom.q.conjugate() * q;
        u.bodyrates.z() = des.yaw_rate; // yaw_rate 前馈走期望值

        fillCommonDebug(des, odom, des_acc, u.q, des_trajectory_yaw, u.thrust);
        recordThrustSample(u.thrust);
        return debug_msg_;
    }

    // 位置环 + 姿态环双环主控: 位置环出加速度/姿态/推力,
    // 姿态环由姿态角误差出期望体轴角速率(含 UDE/TVUDE/TTVUDE/PID)
    quadrotor_msgs::Px4ctrlDebug Attitude_Angular_Control::calculateControlCMD(const Desired_State_t &des,
                                                                               const ctrl_node::Mission_Trigger_t &mission,
                                                                               const ctrl_node::Odom_Data_t &odom,
                                                                               const ctrl_node::Imu_Data_t &imu,
                                                                               Controller_Output_t &u,
                                                                               ros::Time &t)
    {
        handleMissionTrigger(mission, odom, t);

        // 位置环: PD 前馈 + 扰动估计 → 安全限幅后的期望总加速度(含 g)
        Eigen::Vector3d d_acc(0.0, 0.0, 0.0);
        Eigen::Vector3d T_pose(0.0, 0.0, 0.0);
        Eigen::Vector3d Tdot_pose(0.0, 0.0, 0.0);
        Eigen::Vector3d des_acc = computePositionLoopAccel(des, odom, mission, t,
                                                           /*pid_integral_always=*/false,
                                                           d_acc, T_pose, Tdot_pose);

        // odom 欧拉角本周期只算一次, eul_err 与各估计器分支复用
        const Eigen::Vector3d odom_euler = q2euler(odom.q);

        /* Obtain initial state-attitude */
        if (is_first_in_att_control)
        {
            last_att_time = t;
            init_att_euler_ = odom_euler; // 首拍缓存, 任务期间不再重算
        }

        double current_time = getTimeDiff();
        (void)getTimefuture(); // time_future_ 暂未参与计算, 保留读取以维持接口
        Eigen::Vector3d future_velo = getfuturevelocity();
        Eigen::Vector3d initial_velo = getinitialvelocity();
        // 基于速度方向计算 YAW(比位置差更平滑,延迟小)
        double des_trajectory_yaw = calculate_yaw_velo(current_time, future_velo, initial_velo);

        double roll = 0.0;
        double pitch = 0.0;
        accelerationToRollPitch(des_acc, des_trajectory_yaw, roll, pitch);
        u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

        Eigen::Quaterniond q = Eigen::AngleAxisd(des_trajectory_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        u.q = imu.q * odom.q.conjugate() * q;

        // 姿态环
        const Eigen::Vector3d &KAngp = KAngp_;
        const Eigen::Vector3d &KAngi = KAngi_;
        const Eigen::Vector3d &KAngd = KAngd_;

        const double &t_att_min = t_att_min_;
        const double &t_att_max = t_att_max_;
        const double &t_min = t_min_; // TVUDE 姿态分支沿用位置环 t 过渡范围(原位置环函数级别名)
        const double &t_max = t_max_;
        const Eigen::Vector3d &Tatt = Tatt_; // Att-UDE参数
        const Eigen::Vector3d &Tatt_min = Tatt_min_;
        const Eigen::Vector3d &Tatt_max = Tatt_max_;

        Eigen::Vector3d eul_err = Eigen::Vector3d(roll - odom_euler.x(), pitch - odom_euler.y(), std::remainder(des_trajectory_yaw - odom_euler.z(), 2 * M_PI));

        Eigen::Vector3d des_br(0.0, 0.0, 0.0);
        Eigen::Vector3d T_euler(0.0, 0.0, 0.0);
        Eigen::Vector3d Tdot_euler(0.0, 0.0, 0.0);

        double attitude_dt = 0.0;
        const bool attitude_dt_valid =
            getSafeControlDt(t, last_att_time, attitude_dt);
        Eigen::Vector3d eul_err_rate = Eigen::Vector3d::Zero();
        if (attitude_dt_valid)
        {
            eul_err_rate = (eul_err - last_eul_err) / attitude_dt;
        }
        des_br = des.omg + KAngp.asDiagonal() * eul_err +
                 KAngd.asDiagonal() * eul_err_rate;
        last_eul_err = eul_err;

        // 初始化估计器中的加速度 d_br 为零。
        Eigen::Vector3d d_br = Eigen::Vector3d(0.0, 0.0, 0.0);

        // 0:UDE 1:TVUDE 2:PD 3:TTVUDE 4:PID —— 按 estimator_type 选择姿态扰动估计
        if (param_.estimator_type == 1) // 1:TVUDE
        {
            if (mission.active)
            {
                if (is_first_in_att_pubtrigger)
                {
                    init_att_TTV_t = t;
                    is_first_in_att_pubtrigger = false;
                }
                // 累积误差清零
                if (is_first_in_att_control)
                {
                    u0_integral_att = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_att_control = false;
                }
                else
                {
                    if (!attitude_dt_valid)
                    {
                        u0_integral_att.setZero();
                        ROS_WARN_THROTTLE(1.0, "[CTRL]: invalid TVUDE attitude dt; estimator reset for this cycle.");
                    }
                    else
                    {
                        const double t_rel = (t - init_att_TTV_t).toSec();
                        const TVaryingT3 tv = evalTVarying(t_rel, t_min, t_max, Tatt_min, Tatt_max);
                        d_br = odom_euler.cwiseQuotient(tv.T) -
                               init_att_euler_.cwiseQuotient(Tatt_max);
                        u0_integral_att +=
                            (odom_euler.cwiseProduct(tv.dInvT_dt) +
                             des_br.cwiseQuotient(tv.T)) * attitude_dt;
                        d_br -= u0_integral_att;
                        T_euler = tv.T;
                        Tdot_euler = tv.dInvT_dt;
                    }
                }
            }
        }
        else if (param_.estimator_type == 0) // 0:UDE
        {
            if (mission.active)
            {
                if (is_first_in_att_pubtrigger)
                {
                    init_att_TTV_t = t;
                    is_first_in_att_pubtrigger = false;
                }
                // 累积误差清零
                if (is_first_in_att_control)
                {
                    u0_integral_att = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_att_control = false;
                }
                else
                {
                    if (!attitude_dt_valid)
                    {
                        u0_integral_att.setZero();
                        ROS_WARN_THROTTLE(1.0, "[CTRL]: invalid UDE attitude dt; estimator reset for this cycle.");
                    }
                    else
                    {
                        u0_integral_att += des_br * attitude_dt;
                        d_br = (odom_euler - init_att_euler_ - u0_integral_att)
                                   .cwiseQuotient(Tatt);
                        T_euler = Tatt;
                    }
                }
            }
        }
        // 2:PD —— 无姿态扰动估计, d_br 保持 0(默认), last_att_time 由末尾统一更新。
        else if (param_.estimator_type == 3) // 3:TTVUDE
        {
            if (mission.active)
            {
                if (is_first_in_att_pubtrigger)
                {
                    init_att_TTV_t = t;
                    is_first_in_att_pubtrigger = false;
                }
                // 累积误差清零
                if (is_first_in_att_control)
                {
                    u0_integral_att = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_att_control = false;
                }
                else
                {
                    if (!attitude_dt_valid)
                    {
                        u0_integral_att.setZero();
                        ROS_WARN_THROTTLE(1.0, "[CTRL]: invalid TTVUDE attitude dt; estimator reset for this cycle.");
                    }
                    else
                    {
                        const double t_rel = (t - init_att_TTV_t).toSec();
                        const TVaryingT3 tv = evalTVarying(t_rel, t_att_min, t_att_max, Tatt_min, Tatt_max);
                        d_br = odom_euler.cwiseQuotient(tv.T) -
                               init_att_euler_.cwiseQuotient(Tatt_max);
                        u0_integral_att +=
                            (odom_euler.cwiseProduct(tv.dInvT_dt) +
                             des_br.cwiseQuotient(tv.T)) * attitude_dt;
                        d_br -= u0_integral_att;
                        T_euler = tv.T;
                        Tdot_euler = tv.dInvT_dt;
                    }
                }
            }
        }
        else if (param_.estimator_type == 4) // 4: PID
        {
            if (mission.active)
            {
                if (is_first_in_att_pubtrigger)
                {
                    init_att_TTV_t = t;
                    is_first_in_att_pubtrigger = false;
                }
                // 累积误差清零
                if (is_first_in_att_control)
                {
                    eul_integral_ = Eigen::Vector3d(0.0, 0.0, 0.0);
                    is_first_in_att_control = false;
                }
                else
                {
                    double dt = 0.0;
                    const bool dt_valid = getSafeControlDt(t, last_att_time, dt);

                    Eigen::Vector3d eul_att;
                    eul_att << roll - odom_euler.x(), pitch - odom_euler.y(),
                        std::remainder(des_trajectory_yaw - odom_euler.z(), 2.0 * M_PI);

                    const Eigen::Vector3d i_rate_max(
                        param_.kine_cons.omega_roll_max,
                        param_.kine_cons.omega_pitch_max,
                        param_.kine_cons.omega_yaw_max);

                    if (dt_valid)
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            const double i_output = KAngi(i) * eul_integral_(i);
                            const bool upper_saturated = i_output >= i_rate_max(i) - 1e-9;
                            const bool lower_saturated = i_output <= -i_rate_max(i) + 1e-9;
                            const bool drives_upper = upper_saturated && eul_att(i) > 0.0;
                            const bool drives_lower = lower_saturated && eul_att(i) < 0.0;
                            if (!drives_upper && !drives_lower)
                            {
                                eul_integral_(i) += eul_att(i) * dt;
                            }
                            if (std::abs(KAngi(i)) > 1e-9)
                            {
                                const double integral_max = i_rate_max(i) / std::abs(KAngi(i));
                                eul_integral_(i) =
                                    std::max(
                                        -integral_max,
                                        std::min(
                                            integral_max,
                                            eul_integral_(i)));
                            }
                        }
                    }

                    d_br = KAngi.asDiagonal() * eul_integral_;
                }
            }
        }

        last_att_time = t; // 统一更新
        des_br -= d_br;
        u.bodyrates += des_br;

        // used for debug
        fillCommonDebug(des, odom, des_acc, q, des_trajectory_yaw, u.thrust);
        // —— 双环模式特有字段 ——
        debug_msg_.d_acc_x = d_acc(0);
        debug_msg_.d_acc_y = d_acc(1);
        debug_msg_.d_acc_z = d_acc(2);

        debug_msg_.T_x = T_pose(0);
        debug_msg_.T_y = T_pose(1);
        debug_msg_.T_z = T_pose(2);
        debug_msg_.Tdot_x = Tdot_pose(0);
        debug_msg_.Tdot_y = Tdot_pose(1);
        debug_msg_.Tdot_z = Tdot_pose(2);

        debug_msg_.cmd_q_x = u.q.x();
        debug_msg_.cmd_q_y = u.q.y();
        debug_msg_.cmd_q_z = u.q.z();
        debug_msg_.cmd_q_w = u.q.w();

        debug_msg_.imu_q_x = imu.q.x();
        debug_msg_.imu_q_y = imu.q.y();
        debug_msg_.imu_q_z = imu.q.z();
        debug_msg_.imu_q_w = imu.q.w();

        debug_msg_.des_bodyrate_x = u.bodyrates.x();
        debug_msg_.des_bodyrate_y = u.bodyrates.y();
        debug_msg_.des_bodyrate_z = u.bodyrates.z();

        debug_msg_.des_br_x = des_br.x();
        debug_msg_.des_br_y = des_br.y();
        debug_msg_.des_br_z = des_br.z();

        debug_msg_.d_br_x = d_br.x();
        debug_msg_.d_br_y = d_br.y();
        debug_msg_.d_br_z = d_br.z();

        debug_msg_.T_roll = T_euler(0);
        debug_msg_.T_pitch = T_euler(1);
        debug_msg_.T_yaw = T_euler(2);
        debug_msg_.Tdot_roll = Tdot_euler(0);
        debug_msg_.Tdot_pitch = Tdot_euler(1);
        debug_msg_.Tdot_yaw = Tdot_euler(2);

        recordThrustSample(u.thrust);
        return debug_msg_;
    }

    // 期望航向解算: yaw_mode=0 固定 0 rad; =1 沿水平轨迹切线。
    // 低速/垂直段保持上一有效航向; 跨 ±π 取最短角位移
    double Attitude_Angular_Control::calculate_yaw_velo(double t_cur,
                                                        Eigen::Vector3d &velo,
                                                        Eigen::Vector3d &initial_velo)
    {
        (void)initial_velo; // kept in the interface for compatibility

        // Fixed-heading mode: keep yaw at 0 rad and suppress yaw-rate feedforward.
        if (param_.yaw_mode == 0)
        {
            constexpr double kFixedYaw = 0.0;
            last_yaw_ = kFixedYaw;
            last_des_yaw_ = kFixedYaw;
            last_des_yaw_t_ = t_cur;
            last_des_yaw_rate_ = 0.0;
            yaw_rate_inited_ = true;
            is_first_in_calculate_yaw = false;
            return kFixedYaw;
        }

        // Trajectory-tangent mode.
        if (is_first_in_calculate_yaw)
        {
            last_yaw_ = 0.0;
            last_des_yaw_ = last_yaw_;
            last_des_yaw_t_ = t_cur;
            last_des_yaw_rate_ = 0.0;
            yaw_rate_inited_ = false;
            is_first_in_calculate_yaw = false;
            return last_yaw_;
        }

        // future_velo is already the trajectory velocity in the world frame.
        const Eigen::Vector3d dirvelo = velo;

        // Update yaw only when the horizontal trajectory direction is reliable.
        // During vertical climb / hold / descent, retain the last valid yaw.
        double yaw_temp = (dirvelo.head<2>().norm() > 0.05)
                              ? std::atan2(dirvelo(1), dirvelo(0))
                              : last_yaw_;

        // Shortest angular displacement across the +/-pi boundary.
        double d_yaw = yaw_temp - last_yaw_;
        d_yaw = std::remainder(d_yaw, 2.0 * M_PI);

        double yaw = last_yaw_ + d_yaw;
        yaw = std::remainder(yaw, 2.0 * M_PI);
        last_yaw_ = yaw;

        // Yaw-rate feedforward from the tangent-yaw reference.
        if (yaw_rate_inited_)
        {
            double dyaw_ff = yaw - last_des_yaw_;
            dyaw_ff = std::remainder(dyaw_ff, 2.0 * M_PI);
            const double dt_ff = t_cur - last_des_yaw_t_;
            last_des_yaw_rate_ = (dt_ff > 1e-4)
                                     ? (dyaw_ff / dt_ff)
                                     : last_des_yaw_rate_;
        }
        else
        {
            last_des_yaw_rate_ = 0.0;
        }

        last_des_yaw_ = yaw;
        last_des_yaw_t_ = t_cur;
        yaw_rate_inited_ = true;

        return yaw;
    }

    // 期望加速度 → 归一化推力 (姿态对齐 des_acc, 故取矢量模长而非 z 分量)
    double Attitude_Angular_Control::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc)
    {
        // 姿态指令使机体推力对齐 des_acc, 所需总加速度是矢量模长;
        // 只用 z 分量会在倾斜时欠补偿推力
        if (!des_acc.allFinite())
        {
            ROS_ERROR_THROTTLE(1.0, "[CTRL]: non-finite desired acceleration; using hover thrust.");
            return param_.thr_map.hover_percentage;
        }

        if (!std::isfinite(thr2acc_) ||
            thr2acc_ < param_.thr_map.thr2acc_min ||
            thr2acc_ > param_.thr_map.thr2acc_max)
        {
            ROS_ERROR_THROTTLE(1.0,
                               "[CTRL]: invalid thr2acc=%.6f; resetting thrust mapping.",
                               thr2acc_);
            resetThrustMapping();
        }

        double throttle_percentage = des_acc.norm() / thr2acc_;
        if (!std::isfinite(throttle_percentage))
        {
            ROS_ERROR_THROTTLE(1.0, "[CTRL]: non-finite thrust command; using hover thrust.");
            throttle_percentage = param_.thr_map.hover_percentage;
        }

        // MAVROS AttitudeTarget 推力须归一化; FSM 发布侧还有同样一道防线
        const double raw_throttle = throttle_percentage;
        throttle_percentage = std::max(0.0, std::min(1.0, throttle_percentage));
        if (std::abs(raw_throttle - throttle_percentage) > 1e-9)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[CTRL]: collective thrust %.3f saturated to %.3f.",
                              raw_throttle, throttle_percentage);
        }
        return throttle_percentage;
    }

    bool Attitude_Angular_Control::estimateThrustModel(const Eigen::Vector3d &est_a, const ctrl_node::Parameter_t &param)
    {
        // 验证飞行保持 accurate_thrust_model=false: 固定悬停映射,
        // 在线 RLS 不会在大姿态瞬态后破坏稳定
        if (!param.thr_map.accurate_thrust_model)
        {
            return false;
        }

        if (!est_a.allFinite())
        {
            ROS_WARN_THROTTLE(1.0, "[CTRL]: skip thrust-model update: IMU acceleration is non-finite.");
            return false;
        }

        if (!std::isfinite(thr2acc_) ||
            thr2acc_ < param.thr_map.thr2acc_min ||
            thr2acc_ > param.thr_map.thr2acc_max || !std::isfinite(P_) || P_ <= 0.0)
        {
            ROS_WARN_THROTTLE(1.0, "[CTRL]: thrust-model state invalid; resetting estimator.");
            resetThrustMapping();
            return false;
        }

        const ros::Time t_now = ros::Time::now();
        while (!timed_thrust_.empty())
        {
            // Choose data 35~45 ms old.
            const std::pair<ros::Time, double> t_t = timed_thrust_.front();
            const double time_passed = (t_now - t_t.first).toSec();
            if (time_passed > 0.045)
            {
                timed_thrust_.pop();
                continue;
            }
            if (time_passed < 0.035)
            {
                return false;
            }

            const double thr = t_t.second;
            timed_thrust_.pop();

            // 近零/饱和油门样本病态, 旧估计器正是在此被带向零/负 thr2acc
            if (!std::isfinite(thr) || thr < 0.10 || thr > 0.90 ||
                est_a.z() <= 0.0 || est_a.z() > 40.0)
            {
                return false;
            }

            const double denom = rho2_ + thr * P_ * thr;
            if (!std::isfinite(denom) || denom <= 1e-9)
            {
                resetThrustMapping();
                return false;
            }

            const double gamma = 1.0 / denom;
            const double K = gamma * P_ * thr;
            const double candidate_thr2acc =
                thr2acc_ + K * (est_a.z() - thr * thr2acc_);
            const double candidate_P = (1.0 - K * thr) * P_ / rho2_;

            if (!std::isfinite(candidate_thr2acc) ||
                candidate_thr2acc < param.thr_map.thr2acc_min ||
                candidate_thr2acc > param.thr_map.thr2acc_max ||
                !std::isfinite(candidate_P) || candidate_P <= 0.0)
            {
                ROS_WARN_THROTTLE(1.0,
                                  "[CTRL]: rejecting unsafe thrust-model update (candidate thr2acc=%.3f).",
                                  candidate_thr2acc);
                resetThrustMapping();
                return false;
            }

            thr2acc_ = candidate_thr2acc;
            P_ = candidate_P;
            debug_msg_.thr2acc = thr2acc_;
            return true;
        }
        return false;
    }

    void Attitude_Angular_Control::resetThrustMapping()
    {
        thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
        // Keep the reset value inside the configured physical envelope.
        thr2acc_ = std::max(param_.thr_map.thr2acc_min,
                            std::min(param_.thr_map.thr2acc_max, thr2acc_));
        P_ = 1e6;
        debug_msg_.thr2acc = thr2acc_;
    }
}
