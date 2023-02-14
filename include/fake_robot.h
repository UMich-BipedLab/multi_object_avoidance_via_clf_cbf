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
 * File:        fake_robot.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: Publish fake robot inekf state and update inekf state with
 *              control commands
*******************************************************************************/
#ifndef FAKE_ROBOT_H
#define FAKE_ROBOT_H

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

#include "pose.h"
#include "point.h"

#include <vector>
#include <cmath>

namespace bipedlab
{

class fake_robot
{
private:
    //initialization functions
    bool getParameters_();

    //main loop function
    void startRobot_();

    //update received control commands
    bool updatePlannerCommand_();

    //update inekf state
    void moveRobot_();

    //publish inekf state
    void publishInEKF_();

    //callback function
    void getPlannerMsgCallBack_(const planner_msgs::State& msg);

    //states
    inekf_msgs::State current_inekf_;
    planner_msgs::State planner_command_;

    // ROS
    ros::NodeHandle nh_;

    //subscriber
    ros::Subscriber planner_msg_sub_;

    //subscribe channel
    std::string planner_msg_topic_;

    //publisher
    ros::Publisher inekf_pub_;

    //frames
    std::string map_frame_;

    //received data
    planner_msgs::State received_planner_command_;

    //thread lock
    boost::mutex inekf_lock_;
    boost::mutex planner_callback_lock_;

    //update flag
    bool planner_command_updated_;

    //publish rate
    double inekf_publish_rate_;
    double receiving_rate_;

    //parameter
    double time_interval_;
public:

    fake_robot(ros::NodeHandle& nh);
    virtual ~fake_robot();

};

} /*  bipedlab */
#endif /* FAKE_ROBOT_H */
