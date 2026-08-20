#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

#include <trajectory_math/rest_to_rest.hpp>

#include <boost/make_shared.hpp>

//=================================================================
// SimOdom Nodelet —— 期望轨迹生成与下发
//   订阅触发/odom, 按 YAML 参数生成 circle / figure-eight 轨迹(起飞→机动→
//   hold→下降→着陆 5 段), 经时空缩放与起始速度混合后, 以 PositionCommand +
//   点位话题发布期望 p/v/a/j 与航向参考。参数全部来自 odom_param.yaml。
//=================================================================
namespace sim_odom
{

  constexpr double kTwoPi = 6.28318530717958647692; // 2π

  // 平滑核(与 ctrl_node 共用): class scope 不允许非成员 using-declaration,
  // 故在 namespace 顶部引入, SimOdom 内各方法以非限定名调用即可查到。
  using trajectory_math::clamp01;
  using trajectory_math::smoothP;
  using trajectory_math::smoothV;
  using trajectory_math::smoothA;
  using trajectory_math::smoothJ;
  using trajectory_math::smoothPIntegral;

  // 握手 id 由 ctrl_node 编码在 trigger 的 header.frame_id("world:<id>")中。
  // 不可用 header.seq: roscpp publish 时会以连接级自增计数覆盖 seq,
  // 订阅端读到的是传输序号而非写入值。解析失败返回 false。
  bool parseHandshakeId(const std::string &frame_id, std::uint32_t &id_out)
  {
    static const std::string kPrefix = "world:";
    if (frame_id.compare(0, kPrefix.size(), kPrefix) != 0)
      return false;
    const std::string digits = frame_id.substr(kPrefix.size());
    if (digits.empty() || digits.size() > 10 ||
        digits.find_first_not_of("0123456789") != std::string::npos)
      return false;
    std::uint64_t value = 0;
    for (char c : digits)
      value = value * 10U + static_cast<std::uint64_t>(c - '0');
    if (value > 0xFFFFFFFFULL)
      return false;
    id_out = static_cast<std::uint32_t>(value);
    return true;
  }

  class SimOdom : public nodelet::Nodelet
  {
  private:
    //---------------------------------------------------------------
    // 内部数据结构: TrajState(轨迹态 p/v/a/j/yaw) / PhaseState(机动相位 s~sddd)
    //---------------------------------------------------------------
    struct TrajState
    {
      Eigen::Vector3d p = Eigen::Vector3d::Zero();
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      Eigen::Vector3d a = Eigen::Vector3d::Zero();
      Eigen::Vector3d j = Eigen::Vector3d::Zero();
      // 期望航向角参考(按轨迹段显式定义, atan2 主值)。各段在
      // evaluateBaseTrajectory 内赋值;timer_callback 再对其做 unwrap,
      // 保证起飞/机动/下降段边界处航向连续无跳变。
      double yaw = 0.0;
    };

    struct PhaseState
    {
      double s;
      double sd;
      double sdd;
      double sddd;
    };

    //---------------------------------------------------------------
    // 成员变量: ROS 接口 / 运行时状态 / 轨迹原点 / 轨迹参数 (均来自 YAML)
    //---------------------------------------------------------------
    std::thread initThread_;
    std::mutex state_mutex_;

    ros::Subscriber uav_odom_sub_;
    ros::Subscriber trigger_sub_;
    ros::Subscriber ctrl_ready_tri_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher time_diff_pub_;
    ros::Publisher time_future_pub_;
    ros::Publisher future_velocity_pub_;
    ros::Publisher initial_velocity_pub_;
    ros::Timer timer_;

    double time_delay = 0.0;

    Eigen::Vector3d uav_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d uav_v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond uav_q = Eigen::Quaterniond::Identity();

    bool odom_received_ = false;
    ros::Time odom_stamp_;
    bool ctrl_ready_trigger_ = false;
    bool trigger_received_ = false;
    bool trajectory_active_ = false;
    ros::Time trigger_received_stamp_;
    std::uint32_t controller_trajectory_id_ = 0U;
    std::uint32_t active_trajectory_id_ = 0U;

    ros::Time last_timer_time_;
    double trajectory_elapsed_ = 0.0;
    bool trajectory_finished_ = false;

    // Yaw reference state (held while horizontal speed is negligible).
    double last_yaw_ref_ = 0.0;
    double yaw_dot_ref_ = 0.0;
    double yaw_origin_ = 0.0;

    // The trajectory origin is frozen when a valid trigger is received.
    Eigen::Vector3d traj_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_velocity_ = Eigen::Vector3d::Zero();

    // Trajectory parameters. Values are defined only in odom_param.yaml.
    int traj_type = 0;              // 1=circle, 2=figure-eight
    int traj_cycles = 0;            // complete cycles during the maneuver segment
    double traj_radius = 0.0;       // circle radius / figure-eight x amplitude [m]
    double traj_amp_y = 0.0;        // figure-eight y amplitude [m]
    double traj_height = 0.0;       // climb height relative to trigger point [m]
    double traj_rise_time = 0.0;    // climb duration [s]
    double traj_duration = 0.0;     // maneuver duration [s]
    double traj_ramp_ratio = 0.0;   // each acceleration/deceleration ramp fraction (0, 0.5)
    double traj_hold_time = 0.0;    // hover after maneuver before descent [s]
    double traj_descend_time = 0.0; // descent duration [s]
    double traj_total_time_ = 0.0;  // wall-clock length of the full timeline [s]

    // Spatial/temporal scaling and lookahead are also YAML-only parameters.
    double path_scale = 0.0;
    double speed_scale = 0.0;
    double lookahead_time = 0.0;
    double yaw_min_vxy_ = 0.0;         // 保留向后兼容(YAML 仍需提供);yaw 已改为按段显式定义,不再使用此阈值
    double init_vel_blend_time_ = 0.0; // startup velocity-blend window [s]; 0 disables

    double max_timer_dt = 0.0;
    double odom_timeout_ = 0.0;
    double trigger_velocity_threshold = 0.0;
    double trigger_handshake_timeout_ = 0.0;

    std::string frame_id_ = "world";

    static double yawFromQuaternion(const Eigen::Quaterniond &q)
    {
      return std::atan2(
          2.0 * (q.w() * q.z() + q.x() * q.y()),
          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    // 平滑核改用公共头 trajectory_math/rest_to_rest.hpp; using-declaration 在 namespace 顶部
    // (class scope 不允许非成员 using), 本类各方法以非限定名 smoothP/clamp01 调用即可。

    //---------------------------------------------------------------
    // 机动段相位轮廓 (evaluatePhaseProfile):
    //   r∈[0,1] 归一化机动时间 → {s, sd, sdd, sddd}
    //   平滑加速 → 匀速巡航 → 平滑减速 (beta=traj_ramp_ratio 为每段斜坡占比)
    //   满足 s(0)=0, s(1)=1, 两端 sd/sdd/sddd=0
    //---------------------------------------------------------------
    static PhaseState evaluatePhaseProfile(double r, double beta)
    {
      PhaseState out{0.0, 0.0, 0.0, 0.0};
      r = clamp01(r);

      const double normalization = 1.0 - beta;

      // Quintic smoothstep used as the normalized phase-speed ramp.
      auto H = [](double u)
      {
        return 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
      };
      auto Hd = [](double u)
      {
        const double one_minus_u = 1.0 - u;
        return 30.0 * u * u * one_minus_u * one_minus_u;
      };
      auto Hdd = [](double u)
      {
        return 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u);
      };
      // Integral of H, with F(0)=0 and F(1)=1/2.
      auto F = [](double u)
      {
        const double u2 = u * u;
        const double u4 = u2 * u2;
        return 2.5 * u4 - 3.0 * u4 * u + u4 * u2;
      };

      if (r < beta)
      {
        const double u = r / beta;
        out.s = beta * F(u) / normalization;
        out.sd = H(u) / normalization;
        out.sdd = Hd(u) / (beta * normalization);
        out.sddd = Hdd(u) / (beta * beta * normalization);
        return out;
      }

      if (r <= 1.0 - beta)
      {
        out.s = (r - 0.5 * beta) / normalization;
        out.sd = 1.0 / normalization;
        out.sdd = 0.0;
        out.sddd = 0.0;
        return out;
      }

      const double u = (1.0 - r) / beta;
      out.s = 1.0 - beta * F(u) / normalization;
      out.sd = H(u) / normalization;
      out.sdd = -Hd(u) / (beta * normalization);
      out.sddd = Hdd(u) / (beta * beta * normalization);
      return out;
    }

    //---------------------------------------------------------------
    // 机动段终点几何 (setManeuverEndXY / maneuverEndYaw):
    //   计算机动段终点(phi=phi_end, 速度衰减为 0)的位置 XY 与切线航向,
    //   供 hold/下降/着陆段复用, 保证各段航向连续。
    //---------------------------------------------------------------
    void setManeuverEndXY(Eigen::Vector3d &p, double phi_end) const
    {
      if (traj_type == 2)
      {
        constexpr double kPsi0 = 0.78539816339744830962; // π/4
        const double psi_end = phi_end + kPsi0;
        p.x() = traj_origin_.x() + traj_radius * (std::sin(psi_end) - 0.70710678118654752440);
        p.y() = traj_origin_.y() + traj_amp_y * (std::sin(2.0 * psi_end) - 1.0);
      }
      else
      {
        p.x() = traj_origin_.x() + traj_radius * std::sin(phi_end);
        p.y() = traj_origin_.y() + traj_radius * (1.0 - std::cos(phi_end));
      }
    }

    // 机动段几何终点(phi = phi_end, 速度因 ramp-down 衰减为 0)处的切线航向。
    // 作为 hold/下降/着陆段保持的期望航向, 使其与机动段终点连续。
    // 两种轨迹几何终点切线均沿 +X(atan2(0,+) = 0), 故返回 0。
    double maneuverEndYaw(double phi_end) const
    {
      if (traj_type == 2)
      {
        constexpr double kPsi0 = 0.78539816339744830962; // π/4
        const double psi_end = phi_end + kPsi0;
        const double vx = traj_radius * std::cos(psi_end);
        const double vy = 2.0 * traj_amp_y * std::cos(2.0 * psi_end);
        return std::atan2(vy, vx);
      }
      else
      {
        const double vx = traj_radius * std::cos(phi_end);
        const double vy = traj_radius * std::sin(phi_end);
        return std::atan2(vy, vx);
      }
    }

    //=================================================================
    // evaluateBaseTrajectory(): 基础轨迹生成 (核心)
    //   按时间 t 分 5 段求 p/v/a/j 与期望航向 yaw:
    //     Seg1 垂直爬升 (yaw≡0)
    //     Seg2 机动段   circle / figure-eight (yaw=切线方向)
    //     Seg3 hold     保持机动终点 (yaw=机动终点航向)
    //     Seg4 垂直下降 (yaw=机动终点航向)
    //     Seg5 着陆保持 (yaw=机动终点航向)
    //=================================================================
    TrajState evaluateBaseTrajectory(double t) const
    {
      TrajState out;

      const double x0 = traj_origin_.x();
      const double y0 = traj_origin_.y();
      const double z0 = traj_origin_.z();
      const double z_top = z0 + traj_height;

      const double t_rise_end = traj_rise_time;
      const double t_man_end = t_rise_end + traj_duration;
      const double t_hold_end = t_man_end + traj_hold_time;
      const double t_desc_end = t_hold_end + traj_descend_time;
      const double phi_end =
          2.0 * 3.14159265358979323846 * static_cast<double>(traj_cycles);

      if (t <= 0.0)
      {
        out.p = traj_origin_;
        return out;
      }

      // Segment 1: vertical climb.
      if (t < t_rise_end)
      {
        const double T = traj_rise_time;
        const double r = clamp01(t / T);
        const double s0 = smoothP(r);
        const double s1 = smoothV(r);
        const double s2 = smoothA(r);
        const double s3 = smoothJ(r);

        out.p << x0, y0, z0 + traj_height * s0;
        out.v << 0.0, 0.0, traj_height / T * s1;
        out.a << 0.0, 0.0, traj_height / (T * T) * s2;
        out.j << 0.0, 0.0, traj_height / (T * T * T) * s3;
        out.yaw = 0.0; // 垂直起飞段:期望航向角恒为 0
        return out;
      }

      // Segment 2: closed circle or figure-eight maneuver.
      if (t < t_man_end)
      {
        const double T = traj_duration;
        const double tm = t - t_rise_end;
        const double r = clamp01(tm / T);

        // P1/P4: keep the geometric phase span fixed at 2*pi*traj_cycles,
        // while redistributing time as ramp-up -> cruise -> ramp-down.
        const PhaseState phase = evaluatePhaseProfile(r, traj_ramp_ratio);
        const double phi = phi_end * phase.s;
        const double phi_d = phi_end / T * phase.sd;
        const double phi_dd = phi_end / (T * T) * phase.sdd;
        const double phi_ddd = phi_end / (T * T * T) * phase.sddd;
        const double phi_d2 = phi_d * phi_d;
        const double phi_d3 = phi_d2 * phi_d;

        if (traj_type == 2)
        {
          // 八字轨迹:相位偏移 psi = phi + psi0 (psi0 = π/4),使起点处切向水平(cos 2ψ=0 → v.y=0)
          // 且沿 +X(cos ψ>0 → v.x>0),即机动起点航向为 0,与垂直爬升段偏航(0)连续。
          // 曲线形状不变,仅整体平移使起点落在 (x0,y0)。
          constexpr double kPsi0 = 0.78539816339744830962; // π/4
          const double psi = phi + kPsi0;
          const double sin_psi = std::sin(psi);
          const double cos_psi = std::cos(psi);
          const double sin_2psi = std::sin(2.0 * psi);
          const double cos_2psi = std::cos(2.0 * psi);
          constexpr double kSinPsi0 = 0.70710678118654752440; // sin(π/4) = √2/2
          constexpr double kSin2Psi0 = 1.0;                   // sin(π/2) = 1

          out.p.x() = x0 + traj_radius * (sin_psi - kSinPsi0);
          out.p.y() = y0 + traj_amp_y * (sin_2psi - kSin2Psi0);

          out.v.x() = traj_radius * cos_psi * phi_d;
          out.v.y() = 2.0 * traj_amp_y * cos_2psi * phi_d;

          out.a.x() = -traj_radius * sin_psi * phi_d2 + traj_radius * cos_psi * phi_dd;
          out.a.y() = -4.0 * traj_amp_y * sin_2psi * phi_d2 + 2.0 * traj_amp_y * cos_2psi * phi_dd;

          out.j.x() = -traj_radius * cos_psi * phi_d3 - 3.0 * traj_radius * sin_psi * phi_d * phi_dd + traj_radius * cos_psi * phi_ddd;
          out.j.y() = -8.0 * traj_amp_y * cos_2psi * phi_d3 - 12.0 * traj_amp_y * sin_2psi * phi_d * phi_dd + 2.0 * traj_amp_y * cos_2psi * phi_ddd;
        }
        else
        {
          // 圆轨迹:从起点 (x0,y0) 沿 +X 方向切入(航向 0),圆心位于 (x0, y0+R)。
          // 机动起点航向与垂直爬升段偏航(0)一致,边界无跳变;整圈航向 = φ,从 0 连续转到 2π。
          const double sin_phi = std::sin(phi);
          const double cos_phi = std::cos(phi);

          out.p.x() = x0 + traj_radius * sin_phi;
          out.p.y() = y0 + traj_radius * (1.0 - cos_phi);

          out.v.x() = traj_radius * cos_phi * phi_d;
          out.v.y() = traj_radius * sin_phi * phi_d;

          out.a.x() = -traj_radius * sin_phi * phi_d2 + traj_radius * cos_phi * phi_dd;
          out.a.y() = traj_radius * cos_phi * phi_d2 + traj_radius * sin_phi * phi_dd;

          out.j.x() = -traj_radius * cos_phi * phi_d3 - 3.0 * traj_radius * sin_phi * phi_d * phi_dd + traj_radius * cos_phi * phi_ddd;
          out.j.y() = -traj_radius * sin_phi * phi_d3 + 3.0 * traj_radius * cos_phi * phi_d * phi_dd + traj_radius * sin_phi * phi_ddd;
        }

        out.p.z() = z_top;
        // 机动段:期望航向角跟随水平切线方向 atan2(vy, vx)。
        // 两种轨迹起点切线均沿 +X(atan2(0,+) = 0),与起飞段终点(0)连续;
        // 终点切线同样沿 +X(见 maneuverEndYaw),与下降段起点连续。
        out.yaw = std::atan2(out.v.y(), out.v.x());
        return out;
      }

      // Segment 3 (P3): hold at the maneuver end point before descent.
      if (t < t_hold_end)
      {
        setManeuverEndXY(out.p, phi_end);
        out.p.z() = z_top;
        out.v.setZero();
        out.a.setZero();
        out.j.setZero();
        out.yaw = maneuverEndYaw(phi_end); // 保持机动段终点航向
        return out;
      }

      // Segment 4: vertical descent at the maneuver end point.
      if (t < t_desc_end)
      {
        const double T = traj_descend_time;
        const double td = t - t_hold_end;
        const double r = clamp01(td / T);
        const double s0 = smoothP(r);
        const double s1 = smoothV(r);
        const double s2 = smoothA(r);
        const double s3 = smoothJ(r);

        setManeuverEndXY(out.p, phi_end);
        out.p.z() = z_top - traj_height * s0;
        out.v << 0.0, 0.0, -traj_height / T * s1;
        out.a << 0.0, 0.0, -traj_height / (T * T) * s2;
        out.j << 0.0, 0.0, -traj_height / (T * T * T) * s3;
        out.yaw = maneuverEndYaw(phi_end); // 保持机动段终点航向,与机动段终点连续
        return out;
      }

      // Segment 5: hold the final landed state.
      setManeuverEndXY(out.p, phi_end);
      out.p.z() = z0;
      out.yaw = maneuverEndYaw(phi_end); // 保持机动段终点航向
      return out;
    }

    //---------------------------------------------------------------
    // evaluateTrajectory(): 轨迹缩放 + 起始速度混合
    //   在基础轨迹上施加: 速度尺度(speed_scale)、空间尺度(path_scale, 绕原点);
    //   起始速度混合: 将触发时刻实测速度 initial_velocity_ 在 [0,init_vel_blend_time_]
    //   内平滑淡出, 消除 t=0 处参考速度跳变(p/a/j 由同一混合导出, 保持一致)。
    //---------------------------------------------------------------
    TrajState evaluateTrajectory(double wall_time) const
    {
      const double nonnegative_time = std::max(0.0, wall_time);
      const double trajectory_time = speed_scale * nonnegative_time;
      TrajState state = evaluateBaseTrajectory(trajectory_time);
      // 第一条命令从触发时实测航向开始，在垂直爬升段用 smoothstep 平滑回到
      // 基础轨迹的 +X 航向；爬升结束后仍保持原有“沿轨迹切线”语义。
      if (trajectory_time < traj_rise_time)
      {
        const double yaw_blend =
            1.0 - smoothP(clamp01(trajectory_time / traj_rise_time));
        state.yaw += yaw_origin_ * yaw_blend;
      }

      // Spatial scaling around the frozen trigger point.
      state.p = traj_origin_ + path_scale * (state.p - traj_origin_);

      // Chain rule for tau = speed_scale * t.
      state.v *= path_scale * speed_scale;
      state.a *= path_scale * speed_scale * speed_scale;
      state.j *= path_scale * speed_scale * speed_scale * speed_scale;

      // Startup velocity blend: fade the trigger-time measured velocity
      // (initial_velocity_) into the trajectory over [0, init_vel_blend_time_] so
      // the reference velocity is continuous at t=0. Without it the residual
      // trigger speed (<= trigger_velocity_threshold) appears as an instantaneous
      // tracking error. Position/acceleration/jerk are derived from the same
      // blend (b = 1 - smoothP) so they remain mutually consistent:
      //   Δv = iv*b,  Δp = iv*∫b,  Δa = iv*b',  Δj = iv*b''.
      // Set init_vel_blend_time_ == 0 to disable (e.g. noisy real-flight odom).
      if (init_vel_blend_time_ > 1e-6)
      {
        const double iv[3] = {initial_velocity_.x(), initial_velocity_.y(),
                              initial_velocity_.z()};
        const double T = init_vel_blend_time_;
        const double tau = std::min(nonnegative_time, T);
        const double r = tau / T;                          // in [0, 1]
        const double b = 1.0 - smoothP(r);                 // 1 -> 0
        const double b_int = tau - T * smoothPIntegral(r); // ∫₀^τ b dt
        const double minus_b_dot = smoothV(r) / T;         // -b'(tau)
        const double minus_b_ddot = smoothA(r) / (T * T);  // -b''(tau)
        for (int k = 0; k < 3; ++k)
        {
          state.p(k) += iv[k] * b_int;
          state.v(k) += iv[k] * b;
          state.a(k) -= iv[k] * minus_b_dot;
          state.j(k) -= iv[k] * minus_b_ddot;
        }
      }
      return state;
    }

    //---------------------------------------------------------------
    // ROS 回调 (Subscribers):
    //   es_trigger_callback         —— 轨迹触发(校验 odom/速度, 冻结原点)
    //   es_ctrl_ready_tri_callback —— 控制器就绪触发
    //   es_uav_odom_callback       —— UAV 里程计缓存
    //---------------------------------------------------------------
    void expirePendingTriggers(const ros::Time &now)
    {
      if (trigger_received_)
      {
        const double age = (now - trigger_received_stamp_).toSec();
        if (!std::isfinite(age) || age < 0.0 ||
            age > trigger_handshake_timeout_)
        {
          trigger_received_ = false;
          trigger_received_stamp_ = ros::Time(0);
          ROS_WARN("[planning] User trigger expired before handshake completed (age=%.3f s).",
                   age);
        }
      }

      // controller-ready 在同一次飞行中持续有效；FSM 会在退出 OFFBOARD、
      // 起飞失败或进入 LAND 时发送显式 cancel，避免跨飞行复用。
    }

    void es_trigger_callback(const geometry_msgs::PoseStampedConstPtr &)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const ros::Time now = ros::Time::now();
      expirePendingTriggers(now);

      if (!odom_received_)
      {
        ROS_ERROR("[planning] Trigger rejected: no odometry has been received.");
        return;
      }

      const double odom_age = (now - odom_stamp_).toSec();
      if (!std::isfinite(odom_age) || odom_age < 0.0 ||
          odom_age > odom_timeout_)
      {
        ROS_ERROR("[planning] Trigger rejected: odometry is stale (age=%.3f s).",
                  odom_age);
        return;
      }

      if (trajectory_active_)
      {
        ROS_WARN("[planning] Trigger rejected: trajectory already active.");
        return;
      }

      if (!uav_p.allFinite() || !uav_v.allFinite())
      {
        ROS_ERROR("[planning] Trigger rejected: odometry contains NaN/Inf.");
        return;
      }

      if (uav_v.norm() > trigger_velocity_threshold)
      {
        ROS_WARN("[planning] Trigger rejected: UAV speed %.3f m/s exceeds %.3f m/s.",
                 uav_v.norm(), trigger_velocity_threshold);
        return;
      }

      traj_origin_ = uav_p;
      initial_velocity_ = uav_v;
      trajectory_elapsed_ = 0.0;
      trajectory_finished_ = false;
      yaw_origin_ = yawFromQuaternion(uav_q);
      last_yaw_ref_ = yaw_origin_;
      yaw_dot_ref_ = 0.0;
      last_timer_time_ = ros::Time();
      trigger_received_ = true;
      trigger_received_stamp_ = now;
      trajectory_active_ = false;

      ROS_INFO("[planning] Trigger accepted at (%.3f, %.3f, %.3f), speed %.3f m/s.",
               traj_origin_.x(), traj_origin_.y(), traj_origin_.z(),
               initial_velocity_.norm());
    }

    void es_ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const ros::Time now = ros::Time::now();
      expirePendingTriggers(now);

      if (msg->header.frame_id == "cancel")
      {
        ctrl_ready_trigger_ = false;
        controller_trajectory_id_ = 0U;
        active_trajectory_id_ = 0U;
        trigger_received_ = false;
        trigger_received_stamp_ = ros::Time(0);
        trajectory_active_ = false;
        last_timer_time_ = ros::Time(0);
        ROS_WARN("[planning] Trigger handshake/active trajectory cancelled by FSM.");
        return;
      }

      // frame_id 携带握手 id("world:<id>"); "cancel" 之外无法解析即拒绝。
      std::uint32_t handshake_id = 0U;
      if (!parseHandshakeId(msg->header.frame_id, handshake_id) ||
          handshake_id == 0U)
      {
        ROS_ERROR("[planning] Controller-ready trigger rejected: no valid trajectory_id in frame_id '%s'.",
                  msg->header.frame_id.c_str());
        return;
      }

      if (trajectory_active_)
      {
        ROS_WARN("[planning] Controller-ready trigger ignored: trajectory already active.");
        return;
      }

      // 若用户 trigger 先到，旧实现会把地面位置永久冻结为轨迹原点。
      // controller-ready 是后到信号时，使用此刻的悬停 odom 重新冻结原点。
      if (trigger_received_)
      {
        const double odom_age = (now - odom_stamp_).toSec();
        const bool odom_valid =
            odom_received_ && std::isfinite(odom_age) &&
            odom_age >= 0.0 && odom_age <= odom_timeout_ &&
            uav_p.allFinite() && uav_v.allFinite() &&
            uav_v.norm() <= trigger_velocity_threshold;

        if (!odom_valid)
        {
          ROS_ERROR("[planning] Pending trigger cancelled: invalid odometry at controller-ready.");
          trigger_received_ = false;
          trigger_received_stamp_ = ros::Time(0);
          ctrl_ready_trigger_ = false;
          return;
        }

        traj_origin_ = uav_p;
        initial_velocity_ = uav_v;
        trajectory_elapsed_ = 0.0;
        trajectory_finished_ = false;
        yaw_origin_ = yawFromQuaternion(uav_q);
        last_yaw_ref_ = yaw_origin_;
        yaw_dot_ref_ = 0.0;
        last_timer_time_ = ros::Time();

        ROS_INFO("[planning] Pending trigger origin refreshed at (%.3f, %.3f, %.3f).",
                 traj_origin_.x(), traj_origin_.y(), traj_origin_.z());
      }

      controller_trajectory_id_ = handshake_id;
      ctrl_ready_trigger_ = true;
      ROS_INFO("[planning] Controller-ready trigger accepted (trajectory_id=%u).",
               static_cast<unsigned int>(controller_trajectory_id_));
    }

    void es_uav_odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      uav_p << msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->pose.pose.position.z;
      uav_v << msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z;
      uav_q.w() = msg->pose.pose.orientation.w;
      uav_q.x() = msg->pose.pose.orientation.x;
      uav_q.y() = msg->pose.pose.orientation.y;
      uav_q.z() = msg->pose.pose.orientation.z;

      const bool quaternion_valid =
          uav_q.coeffs().allFinite() && uav_q.norm() > 1e-6;
      odom_received_ =
          uav_p.allFinite() && uav_v.allFinite() && quaternion_valid;
      if (odom_received_)
      {
        uav_q.normalize();
        odom_stamp_ = ros::Time::now();
      }
      else
      {
        ROS_ERROR_THROTTLE(1.0,
                           "[planning] Invalid odometry rejected (p/v/q contains NaN/Inf or zero quaternion).");
      }
    }

    //---------------------------------------------------------------
    // publishPoint(): 点位消息(PointStamped)发布工具
    //---------------------------------------------------------------
    void publishPoint(const ros::Publisher &publisher,
                      const ros::Time &stamp,
                      const Eigen::Vector3d &value) const
    {
      geometry_msgs::PointStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      msg.point.x = value.x();
      msg.point.y = value.y();
      msg.point.z = value.z();
      publisher.publish(msg);
    }

    //=================================================================
    // timer_callback(): 定时器主回调 (核心)
    //   推进轨迹时钟 → 求 current/future 期望态 → 航向 unwrap(消除 ±π 跳变)
    //   → 组装 PositionCommand(p/v/a/j/yaw/yaw_dot) 并发布
    //   → 发布 time_diff/time_future 及 current/future/initial 点位
    //=================================================================
    void timer_callback(const ros::TimerEvent &)
    {
      TrajState current_state;
      TrajState future_state;
      Eigen::Vector3d initial_velocity_snapshot = Eigen::Vector3d::Zero();
      double current_eval_time = 0.0;
      double future_eval_time = 0.0;
      bool stop_after_publish = false;
      std::uint32_t trajectory_id_snapshot = 0U;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!odom_received_)
        {
          return;
        }

        const ros::Time now = ros::Time::now();
        expirePendingTriggers(now);
        const double odom_age = (now - odom_stamp_).toSec();
        if (!std::isfinite(odom_age) || odom_age < 0.0 ||
            odom_age > odom_timeout_)
        {
          ROS_ERROR_THROTTLE(1.0,
                             "[planning] Odometry stale (age=%.3f s); command publication stopped.",
                             odom_age);
          return;
        }
        double dt = 0.0;

        if (!trajectory_active_)
        {
          if (!ctrl_ready_trigger_ || !trigger_received_)
          {
            return;
          }

          trajectory_active_ = true;
          active_trajectory_id_ = controller_trajectory_id_;
          trajectory_finished_ = false;
          trajectory_elapsed_ = 0.0;
          last_timer_time_ = now;

          // 两路 trigger 都是一次性握手；轨迹运行期间不再依赖它们。
          ctrl_ready_trigger_ = false;
          controller_trajectory_id_ = 0U;
          trigger_received_ = false;
          trigger_received_stamp_ = ros::Time(0);
          ROS_INFO("[planning] Trajectory execution started.");
        }
        else
        {
          dt = (now - last_timer_time_).toSec();
          last_timer_time_ = now;

          if (!std::isfinite(dt) || dt < 0.0 || dt > max_timer_dt)
          {
            ROS_WARN_THROTTLE(1.0,
                              "[planning] Time jump detected (dt=%.6f s); "
                              "trajectory clock frozen for this cycle.",
                              dt);
            dt = 0.0;
          }
          trajectory_elapsed_ += dt;

          // 最终点只发布一次，随后停止 cmd，避免下一次起飞消费陈旧任务。
          if (trajectory_elapsed_ >= traj_total_time_)
          {
            trajectory_elapsed_ = traj_total_time_;
            trajectory_finished_ = true;
            stop_after_publish = true;
          }
        }

        // Positive time_delay advances the evaluated reference trajectory.
        current_eval_time = std::max(0.0, trajectory_elapsed_ + time_delay);
        future_eval_time = current_eval_time + lookahead_time;

        current_state = evaluateTrajectory(current_eval_time);
        future_state = evaluateTrajectory(future_eval_time);
        initial_velocity_snapshot = initial_velocity_;
        trajectory_id_snapshot = active_trajectory_id_;

        // 期望航向角从触发时实测航向平滑过渡到基础轨迹航向(current_state.yaw)，
        // 机动段继续保持原有的轨迹切线方向。
        // 这里对其做 unwrap(相对上一拍 last_yaw_ref_),消除 ±π 跳变,使各段边界航向连续;
        // circle 机动段会连续累加到 2π·N,后续段保持该累加值(mod 2π 即 0,与机动段终点一致)。
        const double prev_yaw = last_yaw_ref_;
        const double dyaw = std::remainder(current_state.yaw - prev_yaw, kTwoPi);
        last_yaw_ref_ = prev_yaw + dyaw;
        if (dt > 1e-6)
        {
          yaw_dot_ref_ = dyaw / dt;
        }
      }

      quadrotor_msgs::PositionCommandPtr cmdMsg =
          boost::make_shared<quadrotor_msgs::PositionCommand>();

      cmdMsg->header.stamp = ros::Time::now();
      cmdMsg->header.frame_id = frame_id_;

      cmdMsg->position.x = current_state.p.x();
      cmdMsg->position.y = current_state.p.y();
      cmdMsg->position.z = current_state.p.z();

      cmdMsg->velocity.x = current_state.v.x();
      cmdMsg->velocity.y = current_state.v.y();
      cmdMsg->velocity.z = current_state.v.z();

      cmdMsg->acceleration.x = current_state.a.x();
      cmdMsg->acceleration.y = current_state.a.y();
      cmdMsg->acceleration.z = current_state.a.z();

      cmdMsg->jerk.x = current_state.j.x();
      cmdMsg->jerk.y = current_state.j.y();
      cmdMsg->jerk.z = current_state.j.z();

      cmdMsg->yaw = last_yaw_ref_;
      cmdMsg->yaw_dot = yaw_dot_ref_;
      cmdMsg->trajectory_id = trajectory_id_snapshot;
      cmdMsg->trajectory_flag =
          quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

      odom_pub_.publish(cmdMsg);

      std_msgs::Float64 time_diff_msg;
      time_diff_msg.data = current_eval_time;
      time_diff_pub_.publish(time_diff_msg);

      std_msgs::Float64 time_future_msg;
      time_future_msg.data = future_eval_time;
      time_future_pub_.publish(time_future_msg);

      const ros::Time stamp = ros::Time::now();
      publishPoint(future_velocity_pub_, stamp, future_state.v);
      publishPoint(initial_velocity_pub_, stamp, initial_velocity_snapshot);

      if (stop_after_publish)
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        trajectory_active_ = false;
        active_trajectory_id_ = 0U;
        last_timer_time_ = ros::Time();
        ROS_INFO("[planning] Trajectory completed; final command published once, then stopped.");
      }
    }

    //---------------------------------------------------------------
    // loadTrajectoryParameters(): 轨迹参数加载与校验 (全部来自 odom_param.yaml)
    //   读取参数 → 逐项有限性/范围校验, 任一缺失或非法则 ROS_FATAL 并返回 false
    //---------------------------------------------------------------
    bool loadTrajectoryParameters(ros::NodeHandle &nh, int &pub_hz)
    {
      bool ok = true;

      // P5: every trajectory-related value is defined in odom_param.yaml.
      // No trajectory defaults are supplied in this C++ file.
      ok = nh.getParam("pub_hz_", pub_hz) && ok;
      ok = nh.getParam("time_delay", time_delay) && ok;

      ok = nh.getParam("traj_type", traj_type) && ok;
      ok = nh.getParam("traj_cycles", traj_cycles) && ok;
      ok = nh.getParam("traj_radius", traj_radius) && ok;
      ok = nh.getParam("traj_amp_y", traj_amp_y) && ok;
      ok = nh.getParam("traj_height", traj_height) && ok;
      ok = nh.getParam("traj_rise_time", traj_rise_time) && ok;
      ok = nh.getParam("traj_duration", traj_duration) && ok;
      ok = nh.getParam("traj_ramp_ratio", traj_ramp_ratio) && ok;
      ok = nh.getParam("traj_hold_time", traj_hold_time) && ok;
      ok = nh.getParam("traj_descend_time", traj_descend_time) && ok;

      ok = nh.getParam("path_scale", path_scale) && ok;
      ok = nh.getParam("speed_scale", speed_scale) && ok;
      ok = nh.getParam("lookahead_time", lookahead_time) && ok;
      ok = nh.getParam("yaw_min_vxy", yaw_min_vxy_) && ok;
      ok = nh.getParam("init_vel_blend_time", init_vel_blend_time_) && ok;
      ok = nh.getParam("trigger_velocity_threshold", trigger_velocity_threshold) && ok;
      ok = nh.getParam("max_timer_dt", max_timer_dt) && ok;
      ok = nh.getParam("odom_timeout", odom_timeout_) && ok;
      ok = nh.getParam("trigger_handshake_timeout", trigger_handshake_timeout_) && ok;

      if (!ok)
      {
        ROS_FATAL("[sim_odom] Missing required trajectory parameter in odom_param.yaml.");
        return false;
      }

      const bool finite_parameters =
          std::isfinite(time_delay) &&
          std::isfinite(traj_radius) && std::isfinite(traj_amp_y) &&
          std::isfinite(traj_height) && std::isfinite(traj_rise_time) &&
          std::isfinite(traj_duration) && std::isfinite(traj_ramp_ratio) &&
          std::isfinite(traj_hold_time) && std::isfinite(traj_descend_time) &&
          std::isfinite(path_scale) && std::isfinite(speed_scale) &&
          std::isfinite(lookahead_time) && std::isfinite(max_timer_dt) &&
          std::isfinite(odom_timeout_) &&
          std::isfinite(trigger_handshake_timeout_) &&
          std::isfinite(trigger_velocity_threshold) && std::isfinite(yaw_min_vxy_) &&
          std::isfinite(init_vel_blend_time_);

      if (!finite_parameters || pub_hz <= 0 ||
          (traj_type != 1 && traj_type != 2) || traj_cycles <= 0 ||
          traj_radius < 0.0 || traj_amp_y < 0.0 || traj_height < 0.0 ||
          traj_rise_time <= 0.0 || traj_duration <= 0.0 ||
          traj_ramp_ratio <= 0.0 || traj_ramp_ratio >= 0.5 ||
          traj_hold_time < 0.0 || traj_descend_time <= 0.0 ||
          path_scale <= 0.0 || speed_scale <= 0.0 || lookahead_time < 0.0 ||
          max_timer_dt <= 0.0 || odom_timeout_ <= 0.0 ||
          trigger_handshake_timeout_ <= 0.0 ||
          trigger_velocity_threshold < 0.0 ||
          yaw_min_vxy_ < 0.0 || init_vel_blend_time_ < 0.0)
      {
        ROS_FATAL("[sim_odom] Invalid trajectory parameter value in odom_param.yaml.");
        return false;
      }

      return true;
    }

    //---------------------------------------------------------------
    // init(): 节点初始化 —— 参数加载 + 订阅/发布/定时器创建
    //---------------------------------------------------------------
    void init(ros::NodeHandle nh)
    {
      int pub_hz = 0;

      if (!loadTrajectoryParameters(nh, pub_hz))
      {
        return;
      }

      nh.param<std::string>("frame_id", frame_id_, "world");

      // Wall-clock length of the full timeline (base time scaled by speed_scale).
      traj_total_time_ = (traj_rise_time + traj_duration + traj_hold_time + traj_descend_time) / speed_scale;

      uav_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
          "uav_odom", 1, &SimOdom::es_uav_odom_callback, this,
          ros::TransportHints().tcpNoDelay());
      ctrl_ready_tri_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
          "ctrl_trigger", 1, &SimOdom::es_ctrl_ready_tri_callback, this,
          ros::TransportHints().tcpNoDelay());
      trigger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
          "trigger", 1, &SimOdom::es_trigger_callback, this,
          ros::TransportHints().tcpNoDelay());

      odom_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);
      time_diff_pub_ = nh.advertise<std_msgs::Float64>("time_diff", 1);
      time_future_pub_ = nh.advertise<std_msgs::Float64>("time_future", 1);
      future_velocity_pub_ =
          nh.advertise<geometry_msgs::PointStamped>("future_velocity", 1);
      initial_velocity_pub_ =
          nh.advertise<geometry_msgs::PointStamped>("initial_velocity", 1);

      timer_ = nh.createTimer(
          ros::Duration(1.0 / static_cast<double>(pub_hz)),
          &SimOdom::timer_callback, this);

      ROS_INFO("[sim_odom] Trajectory publisher initialized: type=%d, cycles=%d, "
               "R=%.3f m, Ay=%.3f m, duration=%.3f s, ramp=%.3f, hold=%.3f s, "
               "path_scale=%.3f, speed_scale=%.3f, lookahead=%.3f s.",
               traj_type, traj_cycles, traj_radius, traj_amp_y, traj_duration,
               traj_ramp_ratio, traj_hold_time, path_scale, speed_scale,
               lookahead_time);
    }

  public:
    //---------------------------------------------------------------
    // Nodelet 公共接口: 构造 / 析构 / onInit
    //---------------------------------------------------------------
    SimOdom() = default;

    ~SimOdom() override
    {
      if (initThread_.joinable())
      {
        initThread_.join();
      }
    }

    void onInit() override
    {
      ros::NodeHandle nh(getMTPrivateNodeHandle());
      initThread_ = std::thread(&SimOdom::init, this, nh);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace sim_odom

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sim_odom::SimOdom, nodelet::Nodelet)
