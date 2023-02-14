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
 * File:        fake_robot.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: Publish fake robot inekf state and update inekf state with
 *              control commands
*******************************************************************************/
#include "fake_robot.h"

namespace bipedlab
{

fake_robot::fake_robot(ros::NodeHandle& nh): nh_(nh), planner_command_updated_(false)
{
    if (!fake_robot::getParameters_())
    {
        debugger::debugTitleTextOutput("[Driver]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[Driver] Not enough parameters: ",
                                    "Using default values", 10, BR, BOLD);
        debugger::debugTitleTextOutput("[Driver]", "", 10, BR, BOLD);
        utils::pressEnterToContinue();
    }
    else
    {
        debugger::debugColorOutput("[Driver] Received all parameters", "", 10, BC);
    }

    // subscribers
    planner_msg_sub_ = nh_.subscribe(planner_msg_topic_, 10,
                                &fake_robot::getPlannerMsgCallBack_, this);

    //publisher
    inekf_pub_ = nh_.advertise<inekf_msgs::State> ("inekf", 1);

    //initialize planner command
    planner_command_.velocity.x = 0;
    planner_command_.velocity.y = 0;
    planner_command_.velocity.z = 0;

    //spinner
    ros::AsyncSpinner spinner(nh.param("num_callback_threads", 1));
    spinner.start();

    //publish inekf message
    boost::thread inekf_pub(&fake_robot::publishInEKF_, this);

    //drive robot
    fake_robot::startRobot_();
}

void fake_robot::startRobot_()
{
    ros::Time last_update_time = ros::Time::now();
    ros::Rate move_rate(1.0 / time_interval_);
    double receiving_duration = 1 / receiving_rate_;
    while (ros::ok())
    {
        if (fake_robot::updatePlannerCommand_())
        {
            //move robot
            fake_robot::moveRobot_();
        }

        move_rate.sleep();
    }
}

bool fake_robot::updatePlannerCommand_()
{
    double vx, vy, omega;
    bool updated = planner_command_updated_;
    if (!planner_command_updated_)
    {
        vx = 0;
        vy = 0;
        omega = 0;
        debugger::debugWarningOutput("[FakeRobot]/[updatePlannerCommand] "
                "No new planner command", "", 10);
    }
    else
    {
        planner_callback_lock_.lock();
        vx = received_planner_command_.velocity.x;
        vy = received_planner_command_.velocity.y;
        omega = received_planner_command_.velocity.z;
        planner_callback_lock_.unlock();
    }
    planner_command_updated_ = false;
    planner_command_.velocity.x = vx;
    planner_command_.velocity.y = vy;
    planner_command_.velocity.z = omega;
    return updated;
}

void fake_robot::moveRobot_()
{
    inekf_lock_.lock();
    pose2d_t robot_pose(current_inekf_.position, current_inekf_.orientation);
    inekf_lock_.unlock();

    double x = robot_pose.x, y = robot_pose.y, theta = robot_pose.theta;
    double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
    double vx = planner_command_.velocity.x, vy = planner_command_.velocity.y;
    double omega = planner_command_.velocity.z;

    double vx_world = vx * cos_theta - vy * sin_theta;
    double vy_world = vx * sin_theta + vy * cos_theta;

    x += vx_world * time_interval_;
    y += vy_world * time_interval_;
    theta += omega * time_interval_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    inekf_lock_.lock();
    current_inekf_.header.frame_id = map_frame_;
    current_inekf_.position.x = x;
    current_inekf_.position.y = y;
    current_inekf_.position.z = 0;
    current_inekf_.orientation = tf2::toMsg(q);
    inekf_lock_.unlock();
}

void fake_robot::publishInEKF_()
{
    ros::Rate publish_rate(inekf_publish_rate_);
    while (ros::ok())
    {
        inekf_lock_.lock();
        current_inekf_.header.stamp = ros::Time::now();
        inekf_pub_.publish(current_inekf_);
        inekf_lock_.unlock();

        publish_rate.sleep();
    }
}

void fake_robot::getPlannerMsgCallBack_(const planner_msgs::State& msg)
{
    planner_callback_lock_.lock();
    received_planner_command_ = msg;
    planner_command_updated_ = true;
    planner_callback_lock_.unlock();
}

bool fake_robot::getParameters_()
{
    std::string title_name("[fake_robot]/[getParameters] ");
    bool received_all = true;

    ros_utils::checkROSParam(nh_, "fake_robot/inekf_publish_rate", inekf_publish_rate_,
            getNameOf(inekf_publish_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/receiving_rate", receiving_rate_,
            getNameOf(receiving_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/time_interval", time_interval_,
            getNameOf(time_interval_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/x", current_inekf_.position.x,
            getNameOf(current_inekf_.position.x), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/y", current_inekf_.position.y,
            getNameOf(current_inekf_.position.y), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/z", current_inekf_.position.z,
            getNameOf(current_inekf_.position.z), title_name, received_all);

    double roll, pitch, yaw;
    ros_utils::checkROSParam(nh_, "fake_robot/roll", roll,
            getNameOf(roll), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/pitch", pitch,
            getNameOf(pitch), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/yaw", yaw,
            getNameOf(yaw), title_name, received_all);

    roll = roll / 180 * M_PI;
    pitch = pitch / 180 * M_PI;
    yaw = yaw / 180 * M_PI;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    current_inekf_.orientation = tf2::toMsg(q);

    ros_utils::checkROSParam(nh_, "fake_robot/map_frame", map_frame_,
        getNameOf(map_frame_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/planner_msg_topic", planner_msg_topic_,
        getNameOf(planner_msg_topic_), title_name, received_all);

    return received_all;
}

fake_robot::~fake_robot() { }

} /* bipedlab */
