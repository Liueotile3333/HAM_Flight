#ifndef __AERO_MODEL_HPP
#define __AERO_MODEL_HPP

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>
#include <vector>
#include <string>

#include "input.hpp"
#include "param.hpp"

namespace Controller
{

    // AeroModel: 升力翼四旋翼气动前馈模块。
    //   外部风场订阅(可选) + 空速/α/β → 六系数(LINEAR 解析 / TABLE 查表)
    //   → 力/力矩 → 世界系比力补偿 compensation_acc_world = -F_aero_world/m。
    //   悬停/低速/无机翼/风场失效时返回全零 (valid=false), 退化为标准重力补偿。
    class AeroModel
    {
    public:
        // 六个气动系数 (系数求解的输出)
        struct AeroCoefficients
        {
            double CL = 0.0;
            double CD = 0.0;
            double CY = 0.0;

            double Cl = 0.0;
            double Cm = 0.0;
            double Cn = 0.0;
        };
        // 气动前馈输出: 世界系比力补偿 + 力/力矩诊断
        struct AeroFeedforward
        {
            // 比力补偿 (世界系) [m/s^2] = -F_aero_world/m;
            // controller: limited += compensation_acc_world; limited.z() += gra
            Eigen::Vector3d compensation_acc_world = Eigen::Vector3d::Zero();

            // 诊断字段, 不直接驱动控制器。
            // TODO(B1): 暂无消费者, 待转发到 debug 话题 (Px4ctrlDebug 需加字段, 跨包改动)。
            Eigen::Vector3d force_world = Eigen::Vector3d::Zero();     // 气动力, 世界系
            Eigen::Vector3d force_body_frd = Eigen::Vector3d::Zero();  // 气动力, 机体 FRD
            Eigen::Vector3d moment_body_frd = Eigen::Vector3d::Zero(); // 气动力矩, 机体 FRD [N·m] (未接入执行器)
            double Va = 0.0;
            double alpha_deg = 0.0;
            double beta_deg = 0.0;
            AeroCoefficients coeff;
            bool valid = false; // 本周期是否真正施加了气动补偿
        };

        // aero/model_type: 0=LINEAR (CL/CD 解析), 1=TABLE (查表插值, 含侧向力/力矩)
        enum class ModelType
        {
            LINEAR = 0,
            TABLE = 1,
        };

        // 公共气动状态 (由 computeFeedforward 公共段填充)
        struct AeroState
        {
            double Va = 0.0;
            double alpha_deg = 0.0;
            double beta_deg = 0.0;
            double qbar = 0.0;

            Eigen::Vector3d v_air_world = Eigen::Vector3d::Zero();
            Eigen::Vector3d v_air_body_flu = Eigen::Vector3d::Zero();
            Eigen::Vector3d v_air_body_frd = Eigen::Vector3d::Zero();
        };

        // 初始化: 缓存参数 + TABLE 加载 + (可选)订阅风场。
        // false = 失败, 调用方必须停止控制节点启动。
        bool init(const ctrl_node::Parameter_t &param,
                  const ros::NodeHandle &nh);

        // 风场消息回调 (世界系风场速度)
        void windVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

        // 计算气动前馈; 任一安全检查不过时返回全零 (valid=false)。
        AeroFeedforward computeFeedforward(const ctrl_node::Odom_Data_t &odom) const;

    private:
        // ---- 系数求解 (LINEAR / TABLE 可替换) ----
        // LINEAR: CL=CL_a·α+CL_0, CD=CD_a·α+CD_0, 其余恒零; 不截断不限幅, 恒返回 true。
        bool solveLinearCoefficients(const AeroState &state,
                                     AeroCoefficients &coeff) const;

        // TABLE: 双线性插值六系数; 表未加载 → 零系数 + false。
        bool solveTableCoefficients(const AeroState &state,
                                    AeroCoefficients &coeff) const;
        // 气动前馈总开关; false 时 computeFeedforward() 恒返回零。
        bool enabled_ = false;
        ModelType model_type_ = ModelType::LINEAR;

        // ---- 气动核心参数 (init() 从 yaml 读取) ----
        double rho_ = 1.225;               // 空气密度 [kg/m^3]
        double S_ = 0.0;                   // 机翼参考面积 [m^2]
        double CL_a_ = 0.0;                // dCL/dα (α 以度为单位)
        double CL_0_ = 0.0;                // α=0 处 CL
        double CD_a_ = 0.0;                // dCD/dα
        double CD_0_ = 0.0;                // α=0 处 CD
        double alpha_incidence_deg_ = 0.0; // 机翼安装角 [deg]
        double v_forward_min_ = 1.0;       // 前飞门最小前向速度 [m/s]
        double forward_ratio_min_ = 0.7;   // 前飞门最小前向占比
        bool use_wind_velocity_ = false;   // 是否使用外部风场
        double wind_timeout_ = 0.20;       // 风场数据最大接受时延 [s]

        // ---- 气动限幅参数 (init() 从 yaml 读取) ----
        double alpha_min_deg_ = 0.0, alpha_max_deg_ = 20.0;
        double beta_min_deg_ = -20.0, beta_max_deg_ = 20.0;
        double CL_min_ = 0.0, CL_max_ = 2.0;
        double CD_min_ = 0.0, CD_max_ = 1.0;
        double CY_min_ = -1.0, CY_max_ = 1.0;
        double Cl_min_ = -1.0, Cl_max_ = 1.0;
        double Cm_min_ = -1.0, Cm_max_ = 1.0;
        double Cn_min_ = -1.0, Cn_max_ = 1.0;

        // 几何参考量 (力矩用)。
        // NOTE(C3): 1.0/0.2 为占位值, 须用真实几何替换后才能信任 TABLE 力矩。
        double b_ = 1.0;     // 翼展 [m]
        double c_bar_ = 0.2; // 平均气动弦 [m]

        // 力/力矩模长上限 (须 > 0)。NOTE(C3): yaml 默认 200/50 为宽松占位。
        double force_max_ = 0.0;  // 气动力模长上限 [N]
        double moment_max_ = 0.0; // 气动力矩模长上限 [N·m]

        // 统一系数校验/限幅 (solver 之后执行; solver 内部不做)
        bool coeffFinite(const AeroCoefficients &c) const;
        void clampCoefficients(AeroCoefficients &c) const;

        // ---- 气动数据库 (仅 TABLE, init() 加载一次, 运行时不读文件) ----
        bool loadAeroTable(const std::string &file_path); // CSV; 成功后 table_loaded_=true
        void resetTableState();                           // 清空表状态, 失败分支复用防残留
        // O(1) 取 (ia, ib) 表项; 越界返回静态零系数。index = ia*nb + ib。
        const AeroCoefficients &tableAt(std::size_t ia, std::size_t ib) const;
        // α-β 双线性插值, 一次返回 6 系数; 查询点由公共段保证在网格范围内。
        AeroCoefficients bilinearInterpolate(double alpha_deg, double beta_deg) const;
        std::vector<double> alpha_grid_;
        std::vector<double> beta_grid_;
        std::vector<AeroCoefficients> coeff_table_; // index = ia*beta_grid_.size() + ib
        bool table_loaded_ = false;

        // 参数缓存 (非拥有指针, init() 赋值后只读)
        const ctrl_node::Parameter_t *param_ = nullptr;

        ros::NodeHandle nh_;
        ros::Subscriber wind_velocity_sub_;

        // Wind velocity in WORLD frame
        Eigen::Vector3d wind_velocity_world_ = Eigen::Vector3d::Zero();
        ros::Time wind_velocity_stamp_;
        bool wind_velocity_received_ = false;
    };

}

#endif
