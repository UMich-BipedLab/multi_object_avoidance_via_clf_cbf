/* Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 */
/*******************************************************************************
 * File:        cbf.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: CLF-CBF system
*******************************************************************************/
#include "cbf.h"

namespace bipedlab
{

clf_cbf::clf_cbf(clf_cbf_param_t clf_cbf_param): clf_cbf_param_(clf_cbf_param)
{

}

Eigen::VectorXd clf_cbf::calculate_f(const Eigen::VectorXd& x)
{
    Eigen::VectorXd f(clf_cbf_param_.x_dim);
    f << 0, 0, 0;
    return f;
}

Eigen::MatrixXd clf_cbf::calculate_g(const Eigen::VectorXd& x)
{
    double r = x(0), delta = x(1), theta = x(2);
    double cosdelta = std::cos(delta), sindelta = std::sin(delta);
    Eigen::MatrixXd g(clf_cbf_param_.x_dim, clf_cbf_param_.u_dim);
    g <<    -cosdelta,      -sindelta,      0,
         sindelta / r,  -cosdelta / r,      1,
                    0,              0,     -1;
    return g;
}

double clf_cbf::calculate_clf(const Eigen::VectorXd& x)
{
    double r = x(0), delta = x(1);
    double clf = (r * r + clf_cbf_param_.gamma * clf_cbf_param_.gamma *
                  std::sin(clf_cbf_param_.beta * delta) *
                  std::sin(clf_cbf_param_.beta * delta)) / 2;
    return clf;
}

Eigen::MatrixXd clf_cbf::calculate_jacobian_clf(const Eigen::VectorXd& x)
{
    double r = x(0), delta = x(1);
    Eigen::MatrixXd jacobian_clf(1, clf_cbf_param_.x_dim);
    jacobian_clf(0, 0) = r;
    jacobian_clf(0, 1) = clf_cbf_param_.beta *
                         clf_cbf_param_.gamma * clf_cbf_param_.gamma *
                         std::sin(clf_cbf_param_.beta * delta) *
                         std::cos(clf_cbf_param_.beta * delta);
    jacobian_clf(0, 2) = 0;
    return jacobian_clf;
}

double clf_cbf::calculate_cbf(const Eigen::VectorXd& x)
{
    std::vector<double> dist_list;

    double cbf = 1.0;
    double r = x(0), delta = x(1), theta = x(2);
    double robot_x = target_position1_.x - r * std::cos(delta + theta);
    double robot_y = target_position1_.y - r * std::sin(delta + theta);
    for (int i = 0; i < obstacles_.size(); ++i)
    {
        double dx = obstacles_[i].x - robot_x;
        double dy = obstacles_[i].y - robot_y;
        double dis1 = dx * dx + dy * dy - obstacles_[i].r * obstacles_[i].r;
        double cost = 0;
        double s = dis1 / distance_threshold_;
        if (dis1 < 0)
        {
            cost = s;
        }
        else if (dis1 < distance_threshold_)
        {
            cost = s * (1 + s - s * s);
        }
        else
        {
            cost = 1;
        }
        cbf *= cost;
    }
    return cbf;
}

Eigen::VectorXd clf_cbf::generateOptimalCLFControl(const Eigen::VectorXd& x)
{
    double r = x(0), delta = x(1);
    double alpha = clf_cbf_param_.alpha, beta = clf_cbf_param_.beta;

    double v_r = clf_cbf_param_.k_r1 * (r / (clf_cbf_param_.k_r2 + r));
    double v_delta = -(2.0/beta) * clf_cbf_param_.k_delta1 *
        (r/ (clf_cbf_param_.k_delta2 + r)) * std::sin(2 * beta * delta);

    double cos_delta = std::cos(delta);
    double sin_delta = std::sin(delta);
    double r_squared = r * r;
    double cos_delta_squared = cos_delta * cos_delta;

    double omega_star =
            -(r * cos_delta * (v_r * sin_delta - r * cos_delta * v_delta)) /
            (alpha + r_squared *cos_delta_squared);

    double vx_star =
            ((alpha * cos_delta * v_r) + (cos_delta * r_squared * v_r) +
             (alpha * r * v_delta * sin_delta)) /
            (alpha + cos_delta_squared * r_squared);

    double vy_star = (alpha * (v_r * sin_delta - r * cos_delta * v_delta)) /
            (alpha + r_squared * cos_delta_squared);

    Eigen::VectorXd u_optimal(clf_cbf_param_.u_dim);
    u_optimal(0) = vx_star;
    u_optimal(1) = vy_star;
    u_optimal(2) = omega_star;
    return u_optimal;
}

Eigen::VectorXd clf_cbf::generateOptimalCLFControl(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
    Eigen::VectorXd clf_command1 = generateOptimalCLFControl(x1);
    Eigen::VectorXd clf_command2 = generateOptimalCLFControl(x2);
    Eigen::VectorXd u_optimal(clf_cbf_param_.u_dim);
    double s = estimated_time_ / clf_cbf_param_.switch_time;
    double switch_coef;
    if (s < 0)
    {
        switch_coef = 0;
    }
    else if (s < 1)
    {
        switch_coef = s * s * (3 - 2 * s);
    }
    else
    {
        switch_coef = 1;
    }
    u_optimal(0) = clf_command1(0) * switch_coef + clf_command2(0) * (1 - switch_coef);
    u_optimal(1) = clf_command1(1) * switch_coef + clf_command2(1) * (1 - switch_coef);
    u_optimal(2) = clf_command1(2) * switch_coef + clf_command2(2) * (1 - switch_coef);
    return u_optimal;
}

Eigen::VectorXd clf_cbf::generateOptimalCBFControl(const Eigen::VectorXd& x)
{
    // return clf control when no obstacle exits
    if (obstacles_.size() == 0)
    {
        return generateOptimalCLFControl(x);
    }

    Eigen::VectorXd f = calculate_f(x);
    Eigen::MatrixXd g = calculate_g(x);
    double V = calculate_clf(x), B = calculate_cbf(x);
    Eigen::MatrixXd jacobian_V = calculate_jacobian_clf(x);
    double LfV = (jacobian_V * f)(0,0);
    Eigen::MatrixXd LgV = jacobian_V * g;

    //calculate jacobian_B
    Eigen::MatrixXd jacobian_B(1, clf_cbf_param_.x_dim);
    Eigen::VectorXd temp_x = x;
    temp_x(0) += clf_cbf_param_.epsilon;
    jacobian_B(0, 0) = (calculate_cbf(temp_x) - B) / clf_cbf_param_.epsilon;

    temp_x = x;
    temp_x(1) += clf_cbf_param_.epsilon;
    jacobian_B(0, 1) = (calculate_cbf(temp_x) - B) / clf_cbf_param_.epsilon;

    temp_x = x;
    temp_x(2) += clf_cbf_param_.epsilon;
    jacobian_B(0, 2) = (calculate_cbf(temp_x) - B) / clf_cbf_param_.epsilon;

    double LfB = (jacobian_B * f)(0,0);
    Eigen::MatrixXd LgB = jacobian_B * g;

    std::vector<double> weights(clf_cbf_param_.u_dim);
    weights[0] = clf_cbf_param_.h1;
    weights[1] = clf_cbf_param_.h2;
    weights[2] = clf_cbf_param_.h3;
    Eigen::VectorXd clf_command = generateOptimalCLFControl(x);
    std::vector<double> u_ref(clf_cbf_param_.u_dim);
    u_ref[0] = clf_command(0);
    u_ref[1] = clf_command(1);
    u_ref[2] = clf_command(2);
    if (x(1) == 0)
    {
        u_ref[2] -= 0.00001;
    }

    quadprogpp::Matrix<double> H, CE, CI;
    quadprogpp::Vector<double> f_, ce0, ci0, u;
    int n = clf_cbf_param_.u_dim + 1;
    H.resize(n, n);
    f_.resize(n);
    for (int i = 0; i < n; ++i)
    {
        f_[i] = 0.0;
        for (int j = 0; j < n; ++j)
        {
            H[i][j] = 0.0;
        }
    }
    for (int i = 0; i < clf_cbf_param_.u_dim; ++i)
    {
        H[i][i] = weights[i];
        f_[i] = -weights[i] * u_ref[i];
    }
    H[clf_cbf_param_.u_dim][clf_cbf_param_.u_dim] = clf_cbf_param_.slack_weight;

    int p = 0;
    CE.resize(n, p);
    ce0.resize(p);

  	int m = 2;
    CI.resize(n, m);
    for (int i = 0; i < clf_cbf_param_.u_dim; ++i)
    {
        CI[i][0] = -LgV(0, i);
        CI[i][1] = LgB(0, i);
    }
    CI[clf_cbf_param_.u_dim][0] = 1;
    CI[clf_cbf_param_.u_dim][1] = 0;

    ci0.resize(m);
    ci0[0] = -LfV - clf_cbf_param_.clf_rate * V;
    ci0[1] = LfB + clf_cbf_param_.cbf_rate * B;
    u.resize(n);

    quadprogpp::solve_quadprog(H, f_, CE, ce0, CI, ci0, u);
    Eigen::VectorXd u_optimal(clf_cbf_param_.u_dim);
    for (int i = 0; i < clf_cbf_param_.u_dim; ++i)
    {
        u_optimal(i) = u[i];
    }

    return u_optimal;
}

Eigen::VectorXd clf_cbf::generateOptimalCBFControl(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
    Eigen::VectorXd clf_command = generateOptimalCLFControl(x1, x2);

    // return clf control when no obstacle exits
    if (obstacles_.size() == 0)
    {
        return clf_command;
    }

    Eigen::VectorXd f = calculate_f(x1);
    Eigen::MatrixXd g = calculate_g(x1);
    double V = calculate_clf(x1), B = calculate_cbf(x1);
    Eigen::MatrixXd jacobian_V = calculate_jacobian_clf(x1);
    double LfV = (jacobian_V * f)(0,0);
    Eigen::MatrixXd LgV = jacobian_V * g;

    //calculate jacobian_B
    Eigen::MatrixXd jacobian_B(1, clf_cbf_param_.x_dim);
    Eigen::VectorXd temp_x = x1;
    temp_x(0) += clf_cbf_param_.epsilon;
    jacobian_B(0, 0) = (calculate_cbf(temp_x) - B) / clf_cbf_param_.epsilon;

    temp_x = x1;
    temp_x(1) += clf_cbf_param_.epsilon;
    jacobian_B(0, 1) = (calculate_cbf(temp_x) - B) / clf_cbf_param_.epsilon;

    temp_x = x1;
    temp_x(2) += clf_cbf_param_.epsilon;
    jacobian_B(0, 2) = (calculate_cbf(temp_x) - B) / clf_cbf_param_.epsilon;

    double LfB = (jacobian_B * f)(0,0);
    Eigen::MatrixXd LgB = jacobian_B * g;

    std::vector<double> weights(clf_cbf_param_.u_dim);
    weights[0] = clf_cbf_param_.h1;
    weights[1] = clf_cbf_param_.h2;
    weights[2] = clf_cbf_param_.h3;
    std::vector<double> u_ref(clf_cbf_param_.u_dim);
    u_ref[0] = clf_command(0);
    u_ref[1] = clf_command(1);
    u_ref[2] = clf_command(2);
    if (x1(1) == 0)
    {
        u_ref[2] -= 0.00001;
    }

    quadprogpp::Matrix<double> H, CE, CI;
    quadprogpp::Vector<double> f_, ce0, ci0, u;
    int n = clf_cbf_param_.u_dim + 1;
    H.resize(n, n);
    f_.resize(n);
    for (int i = 0; i < n; ++i)
    {
        f_[i] = 0.0;
        for (int j = 0; j < n; ++j)
        {
            H[i][j] = 0.0;
        }
    }
    for (int i = 0; i < clf_cbf_param_.u_dim; ++i)
    {
        H[i][i] = weights[i];
        f_[i] = -weights[i] * u_ref[i];
    }
    H[clf_cbf_param_.u_dim][clf_cbf_param_.u_dim] = clf_cbf_param_.slack_weight;

    int p = 0;
    CE.resize(n, p);
    ce0.resize(p);

  	int m = 2;
    CI.resize(n, m);
    for (int i = 0; i < clf_cbf_param_.u_dim; ++i)
    {
        CI[i][0] = -LgV(0, i);
        CI[i][1] = LgB(0, i);
    }
    CI[clf_cbf_param_.u_dim][0] = 1;
    CI[clf_cbf_param_.u_dim][1] = 0;

    ci0.resize(m);
    ci0[0] = -LfV - clf_cbf_param_.clf_rate * V;
    ci0[1] = LfB + clf_cbf_param_.cbf_rate * B;
    u.resize(n);

    quadprogpp::solve_quadprog(H, f_, CE, ce0, CI, ci0, u);
    Eigen::VectorXd u_optimal(clf_cbf_param_.u_dim);
    for (int i = 0; i < clf_cbf_param_.u_dim; ++i)
    {
        u_optimal(i) = u[i];
    }

    return u_optimal;
}

void clf_cbf::setTarget1(double x, double y)
{
    target_position1_.x = x;
    target_position1_.y = y;
}

void clf_cbf::setTarget2(double x, double y)
{
    target_position2_.x = x;
    target_position2_.y = y;
}

void clf_cbf::setEstimatedTime(double t)
{
    estimated_time_ = t;
}

void clf_cbf::setObstacles(const std::vector<obstacle_t>& obstacles)
{
    obstacles_.clear();
    for (int i = 0; i < obstacles.size(); ++i)
    {
        obstacles_.push_back(obstacles[i]);
    }
    double min_dis = 100000;
    if (obstacles_.size() == 1)
    {
        distance_threshold_ = clf_cbf_param_.delta_dis * clf_cbf_param_.delta_dis;
    }
    else
    {
        for (int i = 0; i < obstacles_.size(); ++i)
        {
            point2d_t<double> obs_i(obstacles_[i].x, obstacles_[i].y);
            for (int j = i + 1; j < obstacles_.size(); ++j)
            {
                point2d_t<double> obs_j(obstacles_[j].x, obstacles_[j].y);
                double dis = distance_pts(obs_i, obs_j) - obstacles_[i].r - obstacles_[j].r;
                if (dis < min_dis)
                {
                    min_dis = dis;
                }
            }
        }
        distance_threshold_ = std::max(clf_cbf_param_.delta_dis, min_dis);
        distance_threshold_ = distance_threshold_ * distance_threshold_;
    }
}

Eigen::VectorXd clf_cbf::computeEgoPolarCoordinate(double x, double y, double theta, int id)
{
    Eigen::VectorXd state(3);
    double dx, dy;
    if (id == 1)
    {
        dx = target_position1_.x - x;
        dy = target_position1_.y - y;
    }
    else
    {
        dx = target_position2_.x - x;
        dy = target_position2_.y - y;
    }
    state(0) = std::sqrt(dx * dx + dy * dy);
    state(1) = wrapToPi(std::atan2(dy, dx) - theta);
    state(2) = theta;
    return state;
}

double wrapToPi(double angle)
{
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    return angle;
}

}
