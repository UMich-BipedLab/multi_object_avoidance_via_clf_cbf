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
 * File:        driver.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: ROS driver used in autonomy experiment
*******************************************************************************/
#ifndef DRIVER_H
#define DRIVER_H

#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"

#include "utils/utils.h"
#include "utils/ros_utils.h"
#include "utils/plotting.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "inekf_msgs/State.h"
#include "planner_msgs/State.h"

#include "communication.h"
#include "cbf.h"
#include "pose.h"
#include "point.h"
#include "map_utils.h"

#include <vector>
#include <deque>

namespace bipedlab
{

struct robot_params_t
{
    // velocity
    double vx_upper_bound; // m/s
    double vy_upper_bound; // m/s
    double default_omega; // rad/s
    double step_time_interval;
};

struct obstacle_params_t
{
    double obstacle_threshold;
    double narrow_distance;
};

class Driver
{
private:
    //initialization functions
    bool getParameters_();
    void waitForData_();

    //main loop function (running at 5Hz)
    void driveRobot_();

    //update obstacles information
    void updateObstacles_();
    obstacle_t generate_obstacle_(const std::vector<point2d_t<int>>& bound_points);

    //update sub-goal selection
    void updateSubgoal_(const pose2d_t& current_pose);
    bool getSubgoal_(double scale, point2d_t<double>& subgoal, const pose2d_t& current_pose);
    bool isSafe_(const point2d_t<double>& position);
    bool reachedGoal_(const pose2d_t& current_pose, const point2d_t<double>& goal);

    //publish rviz topics
    void publishToRviz_();

    //publish optimal control command by CLF-CBF QP
    void publishControlCommand_();
    void generateOptimalControl_(const pose2d_t& current_pose,
                                 planner_info_to_controller_t& control_command);
    void publishPlannerMsg_(const planner_info_to_controller_t& control_command);

    //update multilayer map
    bool updateMultilayerMap_();

    //update inekf state
    bool updateInEKF_();
    bool updateInEKF_(inekf_msgs::State& current_state);

    //callback functions
    void getMultiLayerMapCallBack_(const grid_map_msgs::GridMap& msg);
    void getInEKFCallBack_(const inekf_msgs::State& msg);
    void getClickPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg);

    //important classes
    Communication* udp_;
    clf_cbf* clfcbf_;

    point2d_t<double> subgoal1_;
    point2d_t<double> subgoal2_;
    std::vector<obstacle_t> obstacles_;
    bool subgoal1_selected_;
    bool subgoal2_selected_;
    double estimated_time_;

    visualization_msgs::Marker trajectory_marker_;

    //states
    grid_map::GridMap multilayer_map_;
    inekf_msgs::State inekf_state_;
    std::vector<point2d_t<double>> goal_list_;
    int current_goal_id_;

    inekf_msgs::State subgoal_inekf_;

    ros::Time last_time_;
    bool time_initialized_;

    // ROS
    ros::NodeHandle nh_;

    //subscriber
    ros::Subscriber map_sub_;
    ros::Subscriber inekf_sub_;
    ros::Subscriber click_point_sub_;

    //subscribe channel
    std::string multilayer_map_topic_;
    std::string inekf_topic_;

    //publisher
    ros::Publisher planner_command_pub_;
    ros::Publisher marker_array_pub_;

    //frames
    std::string map_frame_;

    //received data
    grid_map::GridMap new_multilayer_map_;
    inekf_msgs::State new_inekf_state_;

    //thread lock
    boost::mutex map_lock_;
    boost::mutex inekf_lock_;
    boost::mutex click_point_lock_;
    boost::mutex obstacle_lock_;
    boost::mutex subgoal_lock_;

    //update flag
    bool multilayer_map_updated_;
    bool inekf_updated_;
    bool goal_received_;

    //publish rate
    double subgoal_update_rate_;
    double reactive_planner_rate_;

    //parameter
    robot_params_t robot_param_;
    clf_cbf_param_t clf_cbf_param_;
    communication_t udp_info_;
    obstacle_params_t obstalce_param_;

    double reached_threshold_;
    double step_length_;
    double safe_distance_;
    double goal_distance_;
    double enlarge_scale_;
    bool smooth_uref_;

public:

    Driver(ros::NodeHandle& nh);
    virtual ~Driver();

};

}

#endif
