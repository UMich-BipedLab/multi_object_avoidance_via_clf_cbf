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
 * File:        test_QP.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: test CLF-CBF QP
*******************************************************************************/
#include "utils/utils.h"
#include "utils/ros_utils.h"
#include "cbf.h"

#include <string>
using namespace bipedlab;

int DEBUG_LEVEL = 0;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_qp");
    ros::NodeHandle nh_("test_qp");
    bool received_all = true;
    clf_cbf_param_t clf_cbf_param_;
    double robot_x, robot_y, robot_theta, target_x, target_y, obs_x, obs_y, obs_r;

    std::string title_name("[TestQP]/[getParameters] ");

    ros_utils::checkROSParam(nh_, "robot_x", robot_x,
            getNameOf(robot_x), title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_y", robot_y,
            getNameOf(robot_y), title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_theta", robot_theta,
            getNameOf(robot_theta), title_name, received_all);
    ros_utils::checkROSParam(nh_, "target_x", target_x,
            getNameOf(target_x), title_name, received_all);
    ros_utils::checkROSParam(nh_, "target_y", target_y,
            getNameOf(target_y), title_name, received_all);
    ros_utils::checkROSParam(nh_, "obs_x", obs_x,
            getNameOf(obs_x), title_name, received_all);
    ros_utils::checkROSParam(nh_, "obs_y", obs_y,
            getNameOf(obs_y), title_name, received_all);
    ros_utils::checkROSParam(nh_, "obs_r", obs_r,
            getNameOf(obs_r), title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/cbf_mode",
            clf_cbf_param_.cbf_mode,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.cbf_mode),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/x_dim",
            clf_cbf_param_.x_dim,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.x_dim),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/u_dim",
            clf_cbf_param_.u_dim,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.u_dim),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/clf_rate",
            clf_cbf_param_.clf_rate,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.clf_rate),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/cbf_rate",
            clf_cbf_param_.cbf_rate,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.cbf_rate),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/slack_weight",
            clf_cbf_param_.slack_weight,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.slack_weight),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/alpha",
            clf_cbf_param_.alpha,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.alpha),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/beta",
            clf_cbf_param_.beta,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.beta),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/gamma",
            clf_cbf_param_.gamma,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.gamma),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/k_r1",
            clf_cbf_param_.k_r1,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.k_r1),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/k_r2",
            clf_cbf_param_.k_r2,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.k_r2),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/k_delta1",
            clf_cbf_param_.k_delta1,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.k_delta1),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/k_delta2",
            clf_cbf_param_.k_delta2,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.k_delta2),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/epsilon",
            clf_cbf_param_.epsilon,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.epsilon),
            title_name, received_all);

    clf_cbf clfcbf(clf_cbf_param_);
    clfcbf.setTarget1(target_x, target_y);
    std::vector<obstacle_t> obstacles;
    obstacles.push_back(obstacle_t(obs_x, obs_y, obs_r));
    clfcbf.setObstacles(obstacles);
    Eigen::VectorXd state = clfcbf.computeEgoPolarCoordinate(robot_x, robot_y, robot_theta, 1);
    Eigen::VectorXd control = clfcbf.generateOptimalCBFControl(state);
    std::cout << "optimal control:" << std::endl;
    std::cout << control << std::endl;
    return 0;
}
