#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

namespace Ekf
{

    // 状态矩阵(位置，速度)
    Eigen::Vector2d x_x;
    Eigen::Vector2d x_x_;

    Eigen::Vector2d x_y;
    Eigen::Vector2d x_y_;

    Eigen::Vector2d x_z;
    Eigen::Vector2d x_z_;

    // 初始化不确定性协方差矩阵，位置(0,0)的不确定性为1000，速度的不确定性为1000
    Eigen::MatrixXd P_x(2, 2);
    Eigen::MatrixXd P_x_(2, 2);
    Eigen::MatrixXd P_y(2, 2);
    Eigen::MatrixXd P_y_(2, 2);
    Eigen::MatrixXd P_z(2, 2);
    Eigen::MatrixXd P_z_(2, 2);

    // 状态转移矩阵
    Eigen::MatrixXd F(2, 2);
    // 测量矩阵
    Eigen::MatrixXd H(2, 2);
    // 测量协方差矩阵
    Eigen::MatrixXd R(2, 2);
    Eigen::Matrix2d I = MatrixXd::Identity(2, 2);
    // 过程协方差矩阵
    Eigen::MatrixXd Q(2, 2);

    // 变量定义
    Eigen::Vector2d err_x;
    Eigen::Vector2d err_y;
    Eigen::Vector2d err_z;
    Eigen::Vector2d x_x_old;
    Eigen::Vector2d x_y_old;
    Eigen::Vector2d x_z_old;

    // 误差输出-设定帧数
    int _MAX_SEG = 50; // x/0.005
    std::vector<double> error_detect_list(_MAX_SEG);

    std::vector<double> list_cb(double &state_error)
    {

        error_detect_list.erase(error_detect_list.begin());
        error_detect_list.push_back(state_error);
        return error_detect_list;
    }

}