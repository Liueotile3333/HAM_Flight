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
#include <cmath>
#include <mutex>
#include <thread>

namespace sim_odom {

class SimOdom : public nodelet::Nodelet {
private:
  struct TrajState {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    Eigen::Vector3d j = Eigen::Vector3d::Zero();
  };

  std::thread initThread_;
  std::mutex state_mutex_;

  ros::Subscriber uav_odom_sub_;
  ros::Subscriber triger_sub_;
  ros::Subscriber ctrl_ready_tri_sub_;

  ros::Publisher odom_pub;
  ros::Publisher time_diff_pub;
  ros::Publisher current_position_pub;
  ros::Publisher current_velocity_pub;
  ros::Publisher time_future_pub;
  ros::Publisher future_position_pub;
  ros::Publisher future_velocity_pub;
  ros::Publisher initial_velocity_pub;
  ros::Timer timer_;

  // Bias parameters are retained for compatibility with the existing YAML.
  double time_delay = 0.0;
  Eigen::Vector3d pos_bias = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_bias = Eigen::Quaterniond::Identity();

  Eigen::Vector3d uav_p = Eigen::Vector3d::Zero();
  Eigen::Vector3d uav_v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond uav_q = Eigen::Quaterniond::Identity();

  bool odom_received_ = false;
  bool ctrl_ready_triger_ = false;
  bool triger_received_ = false;
  bool trajectory_active_ = false;

  ros::Time last_timer_time_;
  double trajectory_elapsed_ = 0.0;

  // The trajectory origin is frozen when a valid trigger is received.
  Eigen::Vector3d traj_origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d initial_velocity_ = Eigen::Vector3d::Zero();

  // Trajectory parameters.
  int traj_type = 1;             // 1=circle, 2=figure-eight
  int traj_cycles = 1;           // complete cycles during the maneuver segment
  double traj_radius = 0.0;      // circle radius / figure-eight x amplitude [m]
  double traj_amp_y = 0.0;       // figure-eight y amplitude [m]
  double traj_height = 0.0;      // climb height relative to trigger point [m]
  double traj_rise_time = 0.0;   // base trajectory climb duration [s]
  double traj_duration = 0.0;    // base maneuver duration [s]
  double traj_descend_time = 0.0;// base descent duration [s]

  // Spatial and temporal scaling are deliberately separated.
  double path_scale = 1.0;
  double speed_scale = 1.0;
  double lookahead_time = 0.5;

  double max_timer_dt = 0.2;
  double trigger_velocity_threshold = 0.15;

  static double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
  }

  // Seventh-order rest-to-rest time scaling.
  // Position, velocity, acceleration and jerk are all zero/continuous at
  // the segment boundaries.
  static double smoothP(double r) {
    r = clamp01(r);
    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r3 * r;
    return 35.0 * r4 - 84.0 * r4 * r + 70.0 * r4 * r2 - 20.0 * r4 * r3;
  }

  static double smoothV(double r) {
    r = clamp01(r);
    const double one_minus_r = 1.0 - r;
    return 140.0 * r * r * r * one_minus_r * one_minus_r * one_minus_r;
  }

  static double smoothA(double r) {
    r = clamp01(r);
    const double one_minus_r = 1.0 - r;
    return 420.0 * r * r * one_minus_r * one_minus_r * (1.0 - 2.0 * r);
  }

  static double smoothJ(double r) {
    r = clamp01(r);
    return 840.0 * r * (1.0 - r) * (1.0 - 5.0 * r + 5.0 * r * r);
  }

  void setManeuverEndXY(Eigen::Vector3d& p, double phi_end) const {
    if (traj_type == 2) {
      p.x() = traj_origin_.x() + traj_radius * std::sin(phi_end);
      p.y() = traj_origin_.y() + traj_amp_y * std::sin(2.0 * phi_end);
    } else {
      p.x() = traj_origin_.x() + traj_radius * (std::cos(phi_end) - 1.0);
      p.y() = traj_origin_.y() + traj_radius * std::sin(phi_end);
    }
  }

  TrajState evaluateBaseTrajectory(double t) const {
    TrajState out;

    const double x0 = traj_origin_.x();
    const double y0 = traj_origin_.y();
    const double z0 = traj_origin_.z();
    const double z_top = z0 + traj_height;

    const double t_rise_end = traj_rise_time;
    const double t_man_end = t_rise_end + traj_duration;
    const double t_desc_end = t_man_end + traj_descend_time;
    const double phi_end =
        2.0 * 3.14159265358979323846 * static_cast<double>(traj_cycles);

    if (t <= 0.0) {
      out.p = traj_origin_;
      return out;
    }

    // Segment 1: vertical climb.
    if (t < t_rise_end) {
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
      return out;
    }

    // Segment 2: closed circle or figure-eight maneuver.
    if (t < t_man_end) {
      const double T = traj_duration;
      const double tm = t - t_rise_end;
      const double r = clamp01(tm / T);

      const double phi = phi_end * smoothP(r);
      const double phi_d = phi_end / T * smoothV(r);
      const double phi_dd = phi_end / (T * T) * smoothA(r);
      const double phi_ddd = phi_end / (T * T * T) * smoothJ(r);
      const double phi_d2 = phi_d * phi_d;
      const double phi_d3 = phi_d2 * phi_d;

      if (traj_type == 2) {
        const double sin_phi = std::sin(phi);
        const double cos_phi = std::cos(phi);
        const double sin_2phi = std::sin(2.0 * phi);
        const double cos_2phi = std::cos(2.0 * phi);

        out.p.x() = x0 + traj_radius * sin_phi;
        out.p.y() = y0 + traj_amp_y * sin_2phi;

        out.v.x() = traj_radius * cos_phi * phi_d;
        out.v.y() = 2.0 * traj_amp_y * cos_2phi * phi_d;

        out.a.x() = -traj_radius * sin_phi * phi_d2
                    + traj_radius * cos_phi * phi_dd;
        out.a.y() = -4.0 * traj_amp_y * sin_2phi * phi_d2
                    + 2.0 * traj_amp_y * cos_2phi * phi_dd;

        out.j.x() = -traj_radius * cos_phi * phi_d3
                    - 3.0 * traj_radius * sin_phi * phi_d * phi_dd
                    + traj_radius * cos_phi * phi_ddd;
        out.j.y() = -8.0 * traj_amp_y * cos_2phi * phi_d3
                    - 12.0 * traj_amp_y * sin_2phi * phi_d * phi_dd
                    + 2.0 * traj_amp_y * cos_2phi * phi_ddd;
      } else {
        const double sin_phi = std::sin(phi);
        const double cos_phi = std::cos(phi);

        out.p.x() = x0 + traj_radius * (cos_phi - 1.0);
        out.p.y() = y0 + traj_radius * sin_phi;

        out.v.x() = -traj_radius * sin_phi * phi_d;
        out.v.y() = traj_radius * cos_phi * phi_d;

        out.a.x() = -traj_radius * cos_phi * phi_d2
                    - traj_radius * sin_phi * phi_dd;
        out.a.y() = -traj_radius * sin_phi * phi_d2
                    + traj_radius * cos_phi * phi_dd;

        out.j.x() = traj_radius * sin_phi * phi_d3
                    - 3.0 * traj_radius * cos_phi * phi_d * phi_dd
                    - traj_radius * sin_phi * phi_ddd;
        out.j.y() = -traj_radius * cos_phi * phi_d3
                    - 3.0 * traj_radius * sin_phi * phi_d * phi_dd
                    + traj_radius * cos_phi * phi_ddd;
      }

      out.p.z() = z_top;
      return out;
    }

    // Segment 3: vertical descent at the maneuver end point.
    if (t < t_desc_end) {
      const double T = traj_descend_time;
      const double td = t - t_man_end;
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
      return out;
    }

    // Segment 4: hold the final landed state.
    setManeuverEndXY(out.p, phi_end);
    out.p.z() = z0;
    return out;
  }

  TrajState evaluateTrajectory(double wall_time) const {
    const double nonnegative_time = std::max(0.0, wall_time);
    const double trajectory_time = speed_scale * nonnegative_time;
    TrajState state = evaluateBaseTrajectory(trajectory_time);

    // Spatial scaling around the frozen trigger point.
    state.p = traj_origin_ + path_scale * (state.p - traj_origin_);

    // Chain rule for tau = speed_scale * t.
    state.v *= path_scale * speed_scale;
    state.a *= path_scale * speed_scale * speed_scale;
    state.j *= path_scale * speed_scale * speed_scale * speed_scale;
    return state;
  }

  void es_triger_callback(const geometry_msgs::PoseStampedConstPtr&) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!odom_received_) {
      ROS_ERROR("[planning] Trigger rejected: no odometry has been received.");
      return;
    }

    if (!uav_p.allFinite() || !uav_v.allFinite()) {
      ROS_ERROR("[planning] Trigger rejected: odometry contains NaN/Inf.");
      return;
    }

    if (uav_v.norm() > trigger_velocity_threshold) {
      ROS_WARN("[planning] Trigger rejected: UAV speed %.3f m/s exceeds %.3f m/s.",
               uav_v.norm(), trigger_velocity_threshold);
      return;
    }

    traj_origin_ = uav_p;
    initial_velocity_ = uav_v;
    trajectory_elapsed_ = 0.0;
    last_timer_time_ = ros::Time();
    triger_received_ = true;
    trajectory_active_ = false;

    ROS_INFO("[planning] Trigger accepted at (%.3f, %.3f, %.3f), speed %.3f m/s.",
             traj_origin_.x(), traj_origin_.y(), traj_origin_.z(),
             initial_velocity_.norm());
  }

  void es_ctrl_ready_tri_callback(const geometry_msgs::PoseStampedConstPtr&) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ctrl_ready_triger_ = true;
    ROS_INFO("[planning] Controller-ready trigger accepted.");
  }

  void es_uav_odom_callback(const nav_msgs::OdometryConstPtr& msg) {
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

    odom_received_ = uav_p.allFinite() && uav_v.allFinite();
  }

  void publishPoint(const ros::Publisher& publisher,
                    const ros::Time& stamp,
                    const Eigen::Vector3d& value) const {
    geometry_msgs::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.point.x = value.x();
    msg.point.y = value.y();
    msg.point.z = value.z();
    publisher.publish(msg);
  }

  void timer_callback(const ros::TimerEvent&) {
    TrajState current_state;
    TrajState future_state;
    Eigen::Vector3d initial_velocity_snapshot = Eigen::Vector3d::Zero();
    double current_eval_time = 0.0;
    double future_eval_time = 0.0;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (!ctrl_ready_triger_ || !triger_received_ || !odom_received_) {
        return;
      }

      const ros::Time now = ros::Time::now();

      if (!trajectory_active_) {
        trajectory_active_ = true;
        trajectory_elapsed_ = 0.0;
        last_timer_time_ = now;
        ROS_INFO("[planning] Trajectory execution started.");
      } else {
        double dt = (now - last_timer_time_).toSec();
        last_timer_time_ = now;

        if (!std::isfinite(dt) || dt < 0.0 || dt > max_timer_dt) {
          ROS_WARN_THROTTLE(1.0,
                            "[planning] Time jump detected (dt=%.6f s); "
                            "trajectory clock frozen for this cycle.",
                            dt);
          dt = 0.0;
        }
        trajectory_elapsed_ += dt;
      }

      // Positive time_delay advances the evaluated reference trajectory.
      current_eval_time = std::max(0.0, trajectory_elapsed_ + time_delay);
      future_eval_time = current_eval_time + lookahead_time;

      current_state = evaluateTrajectory(current_eval_time);
      future_state = evaluateTrajectory(future_eval_time);
      initial_velocity_snapshot = initial_velocity_;
    }

    quadrotor_msgs::PositionCommandPtr cmdMsg(
        new quadrotor_msgs::PositionCommand());

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

    odom_pub.publish(cmdMsg);

    std_msgs::Float64 time_diff_msg;
    time_diff_msg.data = current_eval_time;
    time_diff_pub.publish(time_diff_msg);

    std_msgs::Float64 time_future_msg;
    time_future_msg.data = future_eval_time;
    time_future_pub.publish(time_future_msg);

    const ros::Time stamp = ros::Time::now();
    publishPoint(current_position_pub, stamp, current_state.p);
    publishPoint(current_velocity_pub, stamp, current_state.v);
    publishPoint(future_position_pub, stamp, future_state.p);
    publishPoint(future_velocity_pub, stamp, future_state.v);
    publishPoint(initial_velocity_pub, stamp, initial_velocity_snapshot);
  }

  bool loadTrajectoryParameters(ros::NodeHandle& nh, int& pub_hz) {
    bool ok = true;
    ok = nh.getParam("pub_hz_", pub_hz) && ok;
    ok = nh.getParam("traj_type", traj_type) && ok;
    ok = nh.getParam("traj_radius", traj_radius) && ok;
    ok = nh.getParam("traj_amp_y", traj_amp_y) && ok;
    ok = nh.getParam("traj_height", traj_height) && ok;
    ok = nh.getParam("traj_rise_time", traj_rise_time) && ok;
    ok = nh.getParam("traj_duration", traj_duration) && ok;
    ok = nh.getParam("traj_descend_time", traj_descend_time) && ok;

    nh.param("traj_cycles", traj_cycles, 1);
    nh.param("path_scale", path_scale, 1.0);
    nh.param("speed_scale", speed_scale, 1.0);
    nh.param("lookahead_time", lookahead_time, 0.5);
    nh.param("trigger_velocity_threshold", trigger_velocity_threshold, 0.15);
    nh.param("max_timer_dt", max_timer_dt, 0.2);

    // Backward-compatible fallback for an old configuration.
    if (!nh.hasParam("path_scale") && nh.hasParam("vel_scale")) {
      nh.getParam("vel_scale", path_scale);
      ROS_WARN("[sim_odom] 'vel_scale' is deprecated; use 'path_scale'.");
    }

    if (!ok) {
      ROS_FATAL("[sim_odom] One or more required trajectory parameters are missing.");
      return false;
    }

    const bool finite_parameters =
        std::isfinite(traj_radius) && std::isfinite(traj_amp_y) &&
        std::isfinite(traj_height) && std::isfinite(traj_rise_time) &&
        std::isfinite(traj_duration) && std::isfinite(traj_descend_time) &&
        std::isfinite(path_scale) && std::isfinite(speed_scale) &&
        std::isfinite(lookahead_time) && std::isfinite(max_timer_dt) &&
        std::isfinite(trigger_velocity_threshold) && std::isfinite(time_delay);

    if (!finite_parameters || pub_hz <= 0 ||
        (traj_type != 1 && traj_type != 2) || traj_cycles <= 0 ||
        traj_radius < 0.0 || traj_amp_y < 0.0 || traj_height < 0.0 ||
        traj_rise_time <= 0.0 || traj_duration <= 0.0 ||
        traj_descend_time <= 0.0 || path_scale <= 0.0 ||
        speed_scale <= 0.0 || lookahead_time < 0.0 ||
        max_timer_dt <= 0.0 || trigger_velocity_threshold < 0.0) {
      ROS_FATAL("[sim_odom] Invalid trajectory parameter value.");
      return false;
    }

    return true;
  }

  void init(ros::NodeHandle nh) {
    int pub_hz = 0;
    double trans_x = 0.0;
    double trans_y = 0.0;
    double trans_z = 0.0;
    double rotat_roll = 0.0;
    double rotat_pitch = 0.0;
    double rotat_yaw = 0.0;

    nh.param("time_delay", time_delay, 0.0);
    nh.param("trans_x", trans_x, 0.0);
    nh.param("trans_y", trans_y, 0.0);
    nh.param("trans_z", trans_z, 0.0);
    nh.param("rotat_roll", rotat_roll, 0.0);
    nh.param("rotat_pitch", rotat_pitch, 0.0);
    nh.param("rotat_yaw", rotat_yaw, 0.0);

    if (!loadTrajectoryParameters(nh, pub_hz)) {
      return;
    }

    pos_bias << trans_x, trans_y, trans_z;
    const Eigen::AngleAxisd roll_angle(rotat_roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(rotat_pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(rotat_yaw, Eigen::Vector3d::UnitZ());
    q_bias = yaw_angle * pitch_angle * roll_angle;

    uav_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "uav_odom", 1, &SimOdom::es_uav_odom_callback, this,
        ros::TransportHints().tcpNoDelay());
    ctrl_ready_tri_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "ctrl_triger", 1, &SimOdom::es_ctrl_ready_tri_callback, this,
        ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "triger", 1, &SimOdom::es_triger_callback, this,
        ros::TransportHints().tcpNoDelay());

    odom_pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);
    time_diff_pub = nh.advertise<std_msgs::Float64>("time_diff", 1);
    time_future_pub = nh.advertise<std_msgs::Float64>("time_future", 1);
    current_position_pub =
        nh.advertise<geometry_msgs::PointStamped>("current_position", 1);
    current_velocity_pub =
        nh.advertise<geometry_msgs::PointStamped>("current_velocity", 1);
    future_position_pub =
        nh.advertise<geometry_msgs::PointStamped>("future_position", 1);
    future_velocity_pub =
        nh.advertise<geometry_msgs::PointStamped>("future_velocity", 1);
    initial_velocity_pub =
        nh.advertise<geometry_msgs::PointStamped>("initial_velocity", 1);

    timer_ = nh.createTimer(
        ros::Duration(1.0 / static_cast<double>(pub_hz)),
        &SimOdom::timer_callback, this);

    ROS_INFO("[sim_odom] Trajectory publisher initialized: type=%d, cycles=%d, "
             "path_scale=%.3f, speed_scale=%.3f, lookahead=%.3f s.",
             traj_type, traj_cycles, path_scale, speed_scale, lookahead_time);
  }

public:
  SimOdom() = default;

  ~SimOdom() override {
    if (initThread_.joinable()) {
      initThread_.join();
    }
  }

  void onInit() override {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(&SimOdom::init, this, nh);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace sim_odom

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sim_odom::SimOdom, nodelet::Nodelet)
