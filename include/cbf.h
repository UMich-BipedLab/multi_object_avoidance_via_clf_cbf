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
 * File:        cbf.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: CLF-CBF system
*******************************************************************************/
#ifndef CBF_H
#define CBF_H

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Eigenvalues>
#include <cmath>

#include "point.h"
#include "QuadProg++.hh"

namespace bipedlab
{

struct clf_cbf_param_t
{
    int cbf_mode; // 1: use one target, 2: use two targets

    int x_dim;
    int u_dim;

    double h1;
    double h2;
    double h3;
    double clf_rate;
    double cbf_rate;
    double slack_weight;
    double alpha;
    double beta;
    double gamma;
    double eps;
    double delta_dis;
    double switch_time;

    int max_ob;

    double k_r1;
    double k_r2;
    double k_delta1;
    double k_delta2;

    double epsilon;
};

struct obstacle_t
{
    double x;
    double y;
    double r;

    obstacle_t(double x_, double y_, double r_): x(x_), y(y_), r(r_) { }
};

class clf_cbf
{
private:
    clf_cbf_param_t clf_cbf_param_;
    point2d_t<double> target_position1_;
    point2d_t<double> target_position2_;
    double estimated_time_;
    std::vector<obstacle_t> obstacles_;
    double distance_threshold_;

public:

    //constructor
    clf_cbf(clf_cbf_param_t clf_cbf_param);

    //calculate f(x)
    Eigen::VectorXd calculate_f(const Eigen::VectorXd& x);

    //calculate g(x)
    Eigen::MatrixXd calculate_g(const Eigen::VectorXd& x);

    //calculate CLF function V(x)
    double calculate_clf(const Eigen::VectorXd& x);

    //calculate jacobian of CLF function
    Eigen::MatrixXd calculate_jacobian_clf(const Eigen::VectorXd& x);

    //cacluate CBF function B(x)
    double calculate_cbf(const Eigen::VectorXd& x); //use target1

    //calculate reference control command
    Eigen::VectorXd generateOptimalCLFControl(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);
    Eigen::VectorXd generateOptimalCLFControl(const Eigen::VectorXd& x);

    //calculate optimal control command
    Eigen::VectorXd generateOptimalCBFControl(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);
    Eigen::VectorXd generateOptimalCBFControl(const Eigen::VectorXd& x);

    //set target position
    void setTarget1(double x, double y);
    void setTarget2(double x, double y);

    //set estimated time
    void setEstimatedTime(double t);

    //set positions and radius of obstacles
    void setObstacles(const std::vector<obstacle_t>& obstacles);

    //calculate robot coordinate in polar coordinate
    Eigen::VectorXd computeEgoPolarCoordinate(double x, double y, double theta, int id);
};

//wrap the angle to [-pi, pi]
double wrapToPi(double angle);

}

#endif
