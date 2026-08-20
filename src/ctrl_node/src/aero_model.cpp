#include "aero_model.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace Controller
{

    bool AeroModel::init(const ctrl_node::Parameter_t &param, const ros::NodeHandle &nh)
    {
        param_ = &param;
        nh_ = nh;

        nh_.param("aero/enabled", enabled_, false);

        if (!enabled_)
        {
            ROS_WARN(
                "[AERO]: aerodynamic feedforward DISABLED. "
                "Controller will run without aerodynamic compensation.");

            // 禁用时跳过加载与订阅, 初始化本身仍视为成功
            return true;
        }

        int model_type_int = 0;
        nh_.param("aero/model_type", model_type_int, 0);

        if (model_type_int == 0)
        {
            model_type_ = ModelType::LINEAR;
        }
        else if (model_type_int == 1)
        {
            model_type_ = ModelType::TABLE;
        }
        else
        {
            ROS_FATAL(
                "[AERO]: invalid aero/model_type=%d; "
                "expected 0=LINEAR or 1=TABLE.",
                model_type_int);

            return false;
        }

        ROS_INFO(
            "[AERO]: aerodynamic feedforward ENABLED, model=%s",
            model_type_ == ModelType::TABLE ? "TABLE" : "LINEAR");

        // 核心参数
        nh_.param("aero/rho", rho_, 1.225);
        nh_.param("aero/S", S_, 0.0);
        nh_.param("aero/CL_a", CL_a_, 0.0);
        nh_.param("aero/CL_0", CL_0_, 0.0);
        nh_.param("aero/CD_a", CD_a_, 0.0);
        nh_.param("aero/CD_0", CD_0_, 0.0);
        nh_.param("aero/alpha_incidence_deg", alpha_incidence_deg_, 0.0);
        nh_.param("aero/v_forward_min", v_forward_min_, 1.0);
        nh_.param("aero/forward_ratio_min", forward_ratio_min_, 0.7);
        nh_.param("aero/use_wind_velocity", use_wind_velocity_, false);
        nh_.param("aero/wind_timeout", wind_timeout_, 0.20);

        // 早失败校验: 统一 return false 由 FSM 拒绝启动
        // (不用 ROS_BREAK: NDEBUG 下为空操作, 且 abort() 会连带杀死共享 manager)
        if (!std::isfinite(v_forward_min_) || v_forward_min_ < 0.0)
        {
            ROS_FATAL("[AERO]: invalid aero/v_forward_min=%.3f m/s", v_forward_min_);
            return false;
        }
        if (!std::isfinite(forward_ratio_min_) || forward_ratio_min_ < 0.0 || forward_ratio_min_ > 1.0)
        {
            ROS_FATAL("[AERO]: invalid aero/forward_ratio_min=%.3f, expected [0, 1]", forward_ratio_min_);
            return false;
        }

        // 限幅参数
        nh_.param("aero/alpha_min_deg", alpha_min_deg_, 0.0);
        nh_.param("aero/alpha_max_deg", alpha_max_deg_, 20.0);
        nh_.param("aero/beta_min_deg", beta_min_deg_, -20.0);
        nh_.param("aero/beta_max_deg", beta_max_deg_, 20.0);
        nh_.param("aero/CL_min", CL_min_, 0.0);
        nh_.param("aero/CL_max", CL_max_, 2.0);
        nh_.param("aero/CD_min", CD_min_, 0.0);
        nh_.param("aero/CD_max", CD_max_, 1.0);
        nh_.param("aero/CY_min", CY_min_, -1.0);
        nh_.param("aero/CY_max", CY_max_, 1.0);
        nh_.param("aero/Cl_min", Cl_min_, -1.0);
        nh_.param("aero/Cl_max", Cl_max_, 1.0);
        nh_.param("aero/Cm_min", Cm_min_, -1.0);
        nh_.param("aero/Cm_max", Cm_max_, 1.0);
        nh_.param("aero/Cn_min", Cn_min_, -1.0);
        nh_.param("aero/Cn_max", Cn_max_, 1.0);
        // 几何参考量 (力矩用)
        nh_.param("aero/b", b_, 1.0);
        nh_.param("aero/c_bar", c_bar_, 0.2);
        // 力/力矩模长上限 (须 > 0, 否则运行时禁用整个气动)
        nh_.param("aero/force_max", force_max_, 0.0);
        nh_.param("aero/moment_max", moment_max_, 0.0);

        // force_max 过松 (> 2mg) 可掩盖错误气动表, 提醒确认而非静默接受
        if (param_->mass > 0.0 && param_->gra > 0.0 &&
            force_max_ > 2.0 * param_->mass * param_->gra)
        {
            ROS_WARN(
                "[AERO]: aero/force_max=%.1f N is %.2f x m*g (mass=%.2f kg); "
                "such a loose bound can mask a wrong aero table — confirm it is intended.",
                force_max_,
                force_max_ / (param_->mass * param_->gra),
                param_->mass);
        }

        // TABLE 显式选择后, 数据库缺失/损坏/覆盖不足均为初始化失败, 不静默退化
        if (model_type_ == ModelType::TABLE)
        {
            std::string table_file;

            nh_.param<std::string>(
                "aero/table_file",
                table_file,
                "");

            if (table_file.empty())
            {
                ROS_FATAL(
                    "[AERO]: TABLE model selected but "
                    "aero/table_file is empty.");

                return false;
            }

            if (!loadAeroTable(table_file))
            {
                ROS_FATAL(
                    "[AERO]: TABLE model initialization failed. "
                    "Flight controller initialization is aborted.");

                return false;
            }

            ROS_INFO(
                "[AERO]: TABLE aerodynamic model initialized successfully.");
        }

        wind_velocity_world_.setZero();
        wind_velocity_stamp_ = ros::Time(0);
        wind_velocity_received_ = false;

        if (use_wind_velocity_)
        {
            wind_velocity_sub_ =
                nh_.subscribe<geometry_msgs::Vector3Stamped>(
                    "wind_velocity",
                    10,
                    &AeroModel::windVelocityCallback,
                    this);

            ROS_INFO(
                "[AERO]: external wind velocity ENABLED.");
        }
        else
        {
            ROS_INFO(
                "[AERO]: external wind velocity DISABLED; "
                "zero-wind assumption is used.");
        }

        return true;
    }

    void AeroModel::windVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        const Eigen::Vector3d wind_new(msg->vector.x, msg->vector.y, msg->vector.z);
        if (!wind_new.allFinite())
        {
            ROS_WARN_THROTTLE(1.0, "[AERO]: invalid wind velocity received.");
            return;
        }
        wind_velocity_world_ = wind_new;
        wind_velocity_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

        wind_velocity_received_ = true;
    }

    // LINEAR: CL/CD 线性, 侧向/力矩系数恒零。
    // 边界截断、有限性、限幅均由公共段统一处理, 故恒返回 true。
    bool AeroModel::solveLinearCoefficients(
        const AeroState &state,
        AeroCoefficients &coeff) const
    {
        coeff.CL =
            CL_a_ * state.alpha_deg +
            CL_0_;

        coeff.CD =
            CD_a_ * state.alpha_deg +
            CD_0_;

        coeff.CY = 0.0;

        coeff.Cl = 0.0;
        coeff.Cm = 0.0;
        coeff.Cn = 0.0;

        return true;
    }

    // TABLE: α-β 双线性插值六系数; 表未加载或查询越界 → 零系数 + false。
    bool AeroModel::solveTableCoefficients(
        const AeroState &state,
        AeroCoefficients &coeff) const
    {
        if (!table_loaded_)
        {
            coeff = AeroCoefficients();
            ROS_WARN_THROTTLE(5.0,
                              "[AERO]: TABLE model selected but aero table not loaded; feedforward disabled.");
            return false;
        }

        if (state.alpha_deg < alpha_grid_.front() - 1e-9 ||
            state.alpha_deg > alpha_grid_.back() + 1e-9 ||
            state.beta_deg < beta_grid_.front() - 1e-9 ||
            state.beta_deg > beta_grid_.back() + 1e-9)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[AERO]: TABLE query outside validated database domain: "
                "alpha=%.3f deg, beta=%.3f deg.",
                state.alpha_deg,
                state.beta_deg);

            coeff = AeroCoefficients();
            return false;
        }

        coeff = bilinearInterpolate(
            state.alpha_deg,
            state.beta_deg);

        return true;
    }

    bool AeroModel::coeffFinite(const AeroCoefficients &c) const
    {
        return std::isfinite(c.CL) && std::isfinite(c.CD) &&
               std::isfinite(c.CY) && std::isfinite(c.Cl) &&
               std::isfinite(c.Cm) && std::isfinite(c.Cn);
    }

    void AeroModel::clampCoefficients(AeroCoefficients &c) const
    {
        c.CL = std::max(CL_min_, std::min(CL_max_, c.CL));
        c.CD = std::max(CD_min_, std::min(CD_max_, c.CD));
        c.CY = std::max(CY_min_, std::min(CY_max_, c.CY));
        c.Cl = std::max(Cl_min_, std::min(Cl_max_, c.Cl));
        c.Cm = std::max(Cm_min_, std::min(Cm_max_, c.Cm));
        c.Cn = std::max(Cn_min_, std::min(Cn_max_, c.Cn));
    }

    void AeroModel::resetTableState()
    {
        alpha_grid_.clear();
        beta_grid_.clear();
        coeff_table_.clear();
        table_loaded_ = false;
    }

    // 从 CSV 加载气动数据库 (仅 init() 调一次)。
    // 每行 8 列: alpha_deg,beta_deg,CL,CD,CY,Cl,Cm,Cn; '#'后为注释, 表头/空行跳过。
    // 要求正交网格: 每个 (alpha,beta) 组合恰好出现一次, 行序不限。
    bool AeroModel::loadAeroTable(const std::string &file_path)
    {
        resetTableState();

        if (file_path.empty())
        {
            ROS_WARN("[AERO]: aero table file path is empty; TABLE model has no data.");
            return false;
        }

        std::ifstream ifs(file_path);
        if (!ifs.is_open())
        {
            ROS_ERROR("[AERO]: failed to open aero table: %s", file_path.c_str());
            return false;
        }

        struct RawRow
        {
            double a;
            double b;
            AeroCoefficients c;
        };
        std::vector<RawRow> rows;
        std::string line;

        std::size_t line_no = 0;
        bool header_seen = false;

        while (std::getline(ifs, line))
        {
            ++line_no;

            // 去掉 '#' 后的行内注释
            const std::size_t hash =
                line.find('#');

            if (hash != std::string::npos)
            {
                line.erase(hash);
            }

            if (line.find_first_not_of(" \t\r\n") ==
                std::string::npos)
            {
                continue;
            }

            // 跳过表头
            if (!header_seen &&
                line.find("alpha_deg") != std::string::npos &&
                line.find("beta_deg") != std::string::npos)
            {
                header_seen = true;
                continue;
            }

            std::istringstream ss(line);

            std::string tok;
            std::vector<double> v;

            while (std::getline(ss, tok, ','))
            {
                try
                {
                    std::size_t used = 0;

                    const double value =
                        std::stod(tok, &used);

                    // stod 之后只允许空白字符
                    if (tok.find_first_not_of(
                            " \t\r\n",
                            used) != std::string::npos)
                    {
                        throw std::invalid_argument(
                            "trailing characters");
                    }

                    v.push_back(value);
                }
                catch (...)
                {
                    ROS_ERROR(
                        "[AERO]: invalid numeric field "
                        "at CSV line %zu: \"%s\"",
                        line_no,
                        line.c_str());

                    return false;
                }
            }

            // 严格 8 列: alpha,beta,CL,CD,CY,Cl,Cm,Cn
            if (v.size() != 8)
            {
                ROS_ERROR(
                    "[AERO]: CSV line %zu has %zu columns; "
                    "expected exactly 8.",
                    line_no,
                    v.size());

                return false;
            }

            for (double x : v)
            {
                if (!std::isfinite(x))
                {
                    ROS_ERROR(
                        "[AERO]: non-finite value "
                        "at CSV line %zu.",
                        line_no);

                    return false;
                }
            }

            RawRow r;

            // 圆整 1e-6, 避免浮点输入噪声造成伪重复断点
            r.a =
                std::round(v[0] * 1e6) / 1e6;

            r.b =
                std::round(v[1] * 1e6) / 1e6;

            r.c.CL = v[2];
            r.c.CD = v[3];
            r.c.CY = v[4];

            r.c.Cl = v[5];
            r.c.Cm = v[6];
            r.c.Cn = v[7];

            rows.push_back(r);
        }

        if (rows.empty())
        {
            ROS_ERROR("[AERO]: aero table has no data rows in %s", file_path.c_str());
            return false;
        }

        // 提取唯一升序断点
        alpha_grid_.reserve(rows.size());
        beta_grid_.reserve(rows.size());
        for (const auto &r : rows)
        {
            alpha_grid_.push_back(r.a);
            beta_grid_.push_back(r.b);
        }
        std::sort(alpha_grid_.begin(), alpha_grid_.end());
        std::sort(beta_grid_.begin(), beta_grid_.end());
        alpha_grid_.erase(std::unique(alpha_grid_.begin(), alpha_grid_.end()), alpha_grid_.end());
        beta_grid_.erase(std::unique(beta_grid_.begin(), beta_grid_.end()), beta_grid_.end());

        const std::size_t na = alpha_grid_.size();
        const std::size_t nb = beta_grid_.size();

        if (na < 2 || nb < 2)
        {
            ROS_ERROR("[AERO]: aero table must contain at least "
                      "2 alpha points x 2 beta points, got %zu x %zu.",
                      na, nb);

            resetTableState();
            return false;
        }

        // 正交网格校验
        if (rows.size() != na * nb)
        {
            ROS_ERROR("[AERO]: aero table is not a regular grid: "
                      "%zu rows vs %zu alphas x %zu betas",
                      rows.size(), na, nb);

            resetTableState();
            return false;
        }

        // 填表并检测重复/缺失
        coeff_table_.assign(na * nb, AeroCoefficients());
        std::vector<char> filled(na * nb, 0);
        for (const auto &r : rows)
        {
            const std::size_t ia = static_cast<std::size_t>(
                std::lower_bound(alpha_grid_.begin(), alpha_grid_.end(), r.a) - alpha_grid_.begin());
            const std::size_t ib = static_cast<std::size_t>(
                std::lower_bound(beta_grid_.begin(), beta_grid_.end(), r.b) - beta_grid_.begin());
            const std::size_t idx = ia * nb + ib;
            if (filled[idx])
            {
                ROS_ERROR("[AERO]: duplicate (alpha=%.3f,beta=%.3f) entry in aero table", r.a, r.b);
                resetTableState();
                return false;
            }
            coeff_table_[idx] = r.c;
            filled[idx] = 1;
        }
        for (char f : filled)
        {
            if (!f)
            {
                ROS_ERROR("[AERO]: incomplete aero table (missing alpha-beta combination)");
                resetTableState();
                return false;
            }
        }

        // 配置包线须被表域完全覆盖, 禁止外推
        if (alpha_min_deg_ < alpha_grid_.front() - 1e-6 ||
            alpha_max_deg_ > alpha_grid_.back() + 1e-6 ||
            beta_min_deg_ < beta_grid_.front() - 1e-6 ||
            beta_max_deg_ > beta_grid_.back() + 1e-6)
        {
            ROS_ERROR(
                "[AERO]: configured aerodynamic envelope "
                "[alpha %.2f, %.2f], [beta %.2f, %.2f] "
                "is not fully covered by table "
                "[alpha %.2f, %.2f], [beta %.2f, %.2f]. "
                "TABLE extrapolation is forbidden.",
                alpha_min_deg_, alpha_max_deg_,
                beta_min_deg_, beta_max_deg_,
                alpha_grid_.front(), alpha_grid_.back(),
                beta_grid_.front(), beta_grid_.back());

            resetTableState();

            return false;
        }

        table_loaded_ = true;

        ROS_INFO("[AERO]: table domain validation passed.");

        return true;
    }

    // O(1) 取表项 (ia, ib); 越界返回静态零系数
    const AeroModel::AeroCoefficients &AeroModel::tableAt(std::size_t ia, std::size_t ib) const
    {
        static const AeroCoefficients kEmpty;
        const std::size_t nb = beta_grid_.size();
        if (coeff_table_.empty() || nb == 0 || ia >= alpha_grid_.size() || ib >= nb)
            return kEmpty;
        return coeff_table_[ia * nb + ib];
    }

    // α-β 双线性插值, 六系数共用同一格子:
    //   C = (1-tβ)·[C00 + tα(C10−C00)] + tβ·[C01 + tα(C11−C01)]
    // 查询点已由公共段截断到网格范围内。
    AeroModel::AeroCoefficients AeroModel::bilinearInterpolate(double alpha_deg, double beta_deg) const
    {
        AeroCoefficients out; // 全零兜底

        const std::size_t na = alpha_grid_.size();
        const std::size_t nb = beta_grid_.size();
        if (coeff_table_.empty() || na == 0 || nb == 0)
            return out;

        const double a = alpha_deg;
        const double b = beta_deg;

        // 定位 α 区间 [ia, ia+1] 与插值比 t_α
        std::size_t ia = 0;
        double ta = 0.0;
        if (na >= 2)
        {
            std::size_t pos = static_cast<std::size_t>(
                std::upper_bound(alpha_grid_.begin(), alpha_grid_.end(), a) - alpha_grid_.begin());
            if (pos == 0)
                pos = 1;
            if (pos > na - 1)
                pos = na - 1;
            ia = pos - 1;
            const double a0 = alpha_grid_[ia], a1 = alpha_grid_[ia + 1];
            ta = (a1 - a0 > 1e-12) ? (a - a0) / (a1 - a0) : 0.0;
        }

        // 定位 β 区间 [ib, ib+1] 与插值比 t_β
        std::size_t ib = 0;
        double tb = 0.0;
        if (nb >= 2)
        {
            std::size_t pos = static_cast<std::size_t>(
                std::upper_bound(beta_grid_.begin(), beta_grid_.end(), b) - beta_grid_.begin());
            if (pos == 0)
                pos = 1;
            if (pos > nb - 1)
                pos = nb - 1;
            ib = pos - 1;
            const double b0 = beta_grid_[ib], b1 = beta_grid_[ib + 1];
            tb = (b1 - b0 > 1e-12) ? (b - b0) / (b1 - b0) : 0.0;
        }

        // 四个角点
        const AeroCoefficients &c00 = tableAt(ia, ib);
        const AeroCoefficients &c10 = tableAt(ia + 1, ib);
        const AeroCoefficients &c01 = tableAt(ia, ib + 1);
        const AeroCoefficients &c11 = tableAt(ia + 1, ib + 1);

        // 标量双线性混合, 对六个成员分别调用
        auto blend = [](double v00, double v10, double v01, double v11, double t_a, double t_b) -> double
        {
            const double c0 = v00 + t_a * (v10 - v00);
            const double c1 = v01 + t_a * (v11 - v01);
            return c0 + t_b * (c1 - c0);
        };

        out.CL = blend(c00.CL, c10.CL, c01.CL, c11.CL, ta, tb);
        out.CD = blend(c00.CD, c10.CD, c01.CD, c11.CD, ta, tb);
        out.CY = blend(c00.CY, c10.CY, c01.CY, c11.CY, ta, tb);
        out.Cl = blend(c00.Cl, c10.Cl, c01.Cl, c11.Cl, ta, tb);
        out.Cm = blend(c00.Cm, c10.Cm, c01.Cm, c11.Cm, ta, tb);
        out.Cn = blend(c00.Cn, c10.Cn, c01.Cn, c11.Cn, ta, tb);

        return out;
    }

    // 气动前馈主流程:
    //   风场/空速/前飞门 → α/β(截断到包线) → 系数求解(LINEAR/TABLE)
    //   → 有限性+限幅 → 力/力矩(FRD) → world → compensation_acc_world = -F_world/m。
    //   controller: limited += compensation_acc_world; limited.z() += gra。
    // 任一安全检查不过 → 全零 (valid=false), 退化为标准重力补偿。
    AeroModel::AeroFeedforward AeroModel::computeFeedforward(const ctrl_node::Odom_Data_t &odom) const
    {
        AeroFeedforward aero; // 默认全零, valid=false

        if (!enabled_)
        {
            return aero;
        }

        if (!param_)
        {
            return aero;
        }

        if (!(rho_ > 0.0) ||
            !(S_ > 0.0) ||
            !(param_->mass > 0.0) ||
            !(force_max_ > 0.0) ||
            !(moment_max_ > 0.0))
        {
            ROS_ERROR_THROTTLE(
                5.0,
                "[AERO]: invalid aero params "
                "(rho/S/mass/force_max/moment_max must be > 0); "
                "feedforward disabled.");

            return aero;
        }
        const bool moment_ok = (b_ > 0.0) && (c_bar_ > 0.0);

        // 风场数据缺失/超时 → 本周期禁用
        bool use_aero_this_cycle = true;
        if (use_wind_velocity_)
        {
            const ros::Time now = ros::Time::now();
            const bool wind_valid = wind_velocity_received_ && !wind_velocity_stamp_.isZero() && (now - wind_velocity_stamp_).toSec() < wind_timeout_;

            if (!wind_valid)
            {
                use_aero_this_cycle = false;
                ROS_WARN_THROTTLE(1.0, "[AERO]: use_wind_velocity=true, "
                                       "but wind velocity is unavailable/stale; "
                                       "falling back to zero-wind assumption.");
            }
        }
        if (!use_aero_this_cycle)
            return aero;

        // 空速 = 地速 - 风速
        const Eigen::Vector3d v_air_world = odom.v - wind_velocity_world_;
        const double Va = v_air_world.norm();
        if (!std::isfinite(Va) || Va <= 1e-6)
            return aero;

        // world → 机体 FLU → 机体 FRD (X前 Y右 Z下)
        const Eigen::Vector3d v_air_body_flu = odom.q.conjugate() * v_air_world;
        Eigen::Vector3d v_air_body_frd;
        v_air_body_frd << v_air_body_flu.x(), -v_air_body_flu.y(), -v_air_body_flu.z();

        // 前飞门: 非前飞状态不补偿 (FLU/FRD 的 x 均为前向)
        const double v_forward = v_air_body_flu.x();
        const double forward_ratio = v_forward / Va;
        if (!(v_forward > v_forward_min_ && forward_ratio > forward_ratio_min_))
            return aero;

        // α = atan2(z_frd, x_frd) + 机翼安装角; β 正值 = 气流从右侧来
        const double alpha_raw = std::atan2(v_air_body_frd.z(), v_air_body_frd.x()) * (180.0 / M_PI) + alpha_incidence_deg_;
        const double beta_raw = std::atan2(v_air_body_frd.y(),
                                           std::sqrt(v_air_body_frd.x() * v_air_body_frd.x() + v_air_body_frd.z() * v_air_body_frd.z())) *
                                (180.0 / M_PI);

        // 截断到 yaml 包线 (solver 不再各自截断)
        const double alpha_used = std::max(alpha_min_deg_, std::min(alpha_max_deg_, alpha_raw));
        const double beta_used = std::max(beta_min_deg_, std::min(beta_max_deg_, beta_raw));

        const double qbar = 0.5 * rho_ * Va * Va;

        AeroState state;
        state.v_air_world = v_air_world;
        state.Va = Va;
        state.v_air_body_flu = v_air_body_flu;
        state.v_air_body_frd = v_air_body_frd;
        state.alpha_deg = alpha_used;
        state.beta_deg = beta_used;
        state.qbar = qbar;

        // 系数求解: 两套模型唯一的分支点
        AeroCoefficients coeff;
        bool coeff_ok = false;
        switch (model_type_)
        {
        case ModelType::LINEAR:
            coeff_ok = solveLinearCoefficients(state, coeff);
            break;
        case ModelType::TABLE:
            coeff_ok = solveTableCoefficients(state, coeff);
            break;
        default:
            coeff_ok = false;
            break;
        }
        // 统一有限性校验 + 系数限幅 (solver 内部不做)
        if (!coeff_ok || !coeffFinite(coeff))
            return aero;
        clampCoefficients(coeff);

        // 公共力/力矩生成 (coeff 已校验限幅; 全部检查通过后才写 aero)。
        // TODO(C1): 本方案取代早期"升力=纯世界Z"简化, 未在 sim/实飞复核,
        //   验证前不要在大α下信任补偿量。
        const double S = S_;
        const double inv_m = 1.0 / param_->mass;

        // L/D/Y [N], 风轴系
        const double L = qbar * S * coeff.CL;
        const double D = qbar * S * coeff.CD;
        const double Y = qbar * S * coeff.CY;
        const Eigen::Vector3d F_wind(-D, Y, -L);

        // wind → FRD 标准气动 DCM。
        // C2: TABLE 数据须与此约定一致 (α=atan2(z,x), β=atan2(y,√(x²+z²)), F_wind=[-D,Y,-L])。
        // C5: DCM 用截断后的 α/β, 与系数自洽。
        const double alpha_rad = state.alpha_deg * (M_PI / 180.0);
        const double beta_rad = state.beta_deg * (M_PI / 180.0);
        const double ca = std::cos(alpha_rad), sa = std::sin(alpha_rad);
        const double cb = std::cos(beta_rad), sb = std::sin(beta_rad);
        Eigen::Matrix3d R_wind2frd;
        R_wind2frd << ca * cb, -ca * sb, -sa,
            sb, cb, 0.0,
            sa * cb, -sa * sb, ca;
        Eigen::Vector3d F_body_frd = R_wind2frd * F_wind;

        // 力矩仅诊断, 未接入执行器; b/c_bar 非法时置零, 不影响升阻力
        Eigen::Vector3d M_body_frd = Eigen::Vector3d::Zero();
        if (moment_ok)
        {
            M_body_frd.x() = qbar * S * b_ * coeff.Cl;
            M_body_frd.y() = qbar * S * c_bar_ * coeff.Cm;
            M_body_frd.z() = qbar * S * b_ * coeff.Cn;
        }

        if (!F_body_frd.allFinite() || !M_body_frd.allFinite())
        {
            ROS_WARN_THROTTLE(5.0,
                              "[AERO]: non-finite force/moment after generation; feedforward disabled.");
            return aero;
        }

        // 模长限幅 (保方向)
        {
            const double fn = F_body_frd.norm();
            if (fn > force_max_ && fn > 1e-9)
                F_body_frd *= force_max_ / fn;
        }
        {
            const double mn = M_body_frd.norm();
            if (mn > moment_max_ && mn > 1e-9)
                M_body_frd *= moment_max_ / mn;
        }

        // FRD → FLU (Y,Z 取反) → world
        const Eigen::Vector3d F_body_flu(F_body_frd.x(), -F_body_frd.y(), -F_body_frd.z());
        const Eigen::Vector3d F_world = odom.q * F_body_flu;

        const Eigen::Vector3d compensation_acc_world = -F_world * inv_m;

        if (!F_world.allFinite() || !compensation_acc_world.allFinite())
        {
            ROS_WARN_THROTTLE(5.0, "[AERO]: non-finite final output; feedforward disabled.");
            return aero;
        }

        aero.force_body_frd = F_body_frd;
        aero.force_world = F_world;
        aero.moment_body_frd = M_body_frd;
        aero.compensation_acc_world = compensation_acc_world;
        aero.Va = state.Va;
        aero.alpha_deg = state.alpha_deg;
        aero.beta_deg = state.beta_deg;
        aero.coeff = coeff;
        aero.valid = true;

        return aero;
    }
}
