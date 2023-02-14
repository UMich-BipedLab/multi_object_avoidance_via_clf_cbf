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
 * File:        simutest.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: ROS driver used in simulation
*******************************************************************************/
#include "simutest.h"

namespace bipedlab
{

Driver::Driver(ros::NodeHandle& nh): nh_(nh),
multilayer_map_updated_(false), goal_received_(false),
subgoal1_selected_(false), subgoal2_selected_(false), current_goal_id_(0),
trajectory_id_(-1), color_id_(0), estimated_time_(0)
{
    if (!Driver::getParameters_())
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
    map_sub_ = nh_.subscribe(multilayer_map_topic_, 10,
                                &Driver::getMultiLayerMapCallBack_, this);
    if (goal_mode_ == 0)
    {
        click_point_sub_ = nh_.subscribe(std::string("/clicked_point"), 10,
                                    &Driver::getClickPointCallBack_, this);
    }

    //publisher
    planner_command_pub_ = nh_.advertise<planner_msgs::State> (
            "planner_commands", 1);
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "MarkerArray", 10);
    trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "Trajectory", 10);
    target_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "Targets", 10);
    local_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("local_map", 1);

    //spinner
    ros::AsyncSpinner spinner(nh_.param("num_callback_threads", 3));
    spinner.start();

    // initialize classes
    udp_ = new Communication(udp_info_);
    clfcbf_ = new clf_cbf(clf_cbf_param_);

    // wait for data arriving
    Driver::waitForData_();

    //update inekf state and multilayer map
    multilayer_map_.setFrameId(map_frame_);
    Driver::updateMultilayerMap_();

    //drive robot
    Driver::driveRobot_();
    cnt_ = 0;

    //execute clf-cbf and publish control command to robot
    Driver::publishControlCommand_();
}

void Driver::driveRobot_()
{
    //update map
    Driver::updateMultilayerMap_();

    //update obstacles
    Driver::updateObstacles_();

    //update subgoal
    Driver::updateSubgoal_();
}

void Driver::updateObstacles_()
{
    debugger::debugColorOutput("[updateObstacles_] ",
                     " ", 5, C);
    std::vector<obstacle_t> new_obstacles;
    grid_map::Size GridMapSize = multilayer_map_.getSize();
    double res = multilayer_map_.getResolution();
    auto& data = multilayer_map_["elevation"];

    //Get occupancy grid map
    std::vector<std::vector<int>> occupancy(GridMapSize(0),
                                      std::vector<int>(GridMapSize(1), 0));
    for (grid_map::GridMapIterator it(multilayer_map_); !it.isPastEnd(); ++it)
    {
        const grid_map::Index id(*it);
        size_t i = it.getLinearIndex();
        if (!std::isnan(data(i)))
        {
            occupancy[id(0)][id(1)] = data(i) > obstalce_param_.obstacle_threshold;
        }
        else
        {
            occupancy[id(0)][id(1)] = 0;
        }
    }

    //Find all the boundary points
    std::vector<std::vector<bool>> visited(GridMapSize(0),
                              std::vector<bool>(GridMapSize(1), false));
    int obstacle_n = 0;
    for (int i = 0; i < GridMapSize(0); ++i)
    {
        for (int j = 0; j < GridMapSize(1); ++j)
        {
            if (occupancy[i][j] == 1 && !visited[i][j])
            {
                std::vector<point2d_t<int>> bound_points;
                std::deque<point2d_t<int>> point_queue;
                point_queue.push_back(point2d_t<int>(i, j));
                visited[i][j] = true;
                while (!point_queue.empty())
                {
                    std::vector<point2d_t<int>> neighbor;
                    getNeighborPoints(point_queue.front(), neighbor,
                        0, GridMapSize(0), 0, GridMapSize(1), 2,
                        obstalce_param_.narrow_distance / res);
                    bool flag = false;
                    for (int k = 0; k < neighbor.size(); ++k)
                    {
                        int px = neighbor[k].x, py = neighbor[k].y;
                        if (!visited[px][py] && occupancy[px][py] == 1)
                        {
                            point_queue.push_back(neighbor[k]);
                            visited[px][py] = true;
                        }
                    }
                    std::vector<point2d_t<int>> neighbor2;
                    getNeighborPoints(point_queue.front(), neighbor2,
                        0, GridMapSize(0), 0, GridMapSize(1), 0, 0);
                    for (int k = 0; k < neighbor2.size(); ++k)
                    {
                        int px = neighbor2[k].x, py = neighbor2[k].y;
                        if (occupancy[px][py] == 1)
                        {
                            flag = true;
                        }
                    }
                    if (flag)
                    {
                        bound_points.push_back(point_queue.front());
                    }
                    point_queue.pop_front();
                }
                if (bound_points.size() > 0)
                {
                    new_obstacles.push_back(Driver::generate_obstacle_(bound_points));
                }
            }
        }
    }

    obstacles_.clear();
    obstacles_ = new_obstacles;
}

obstacle_t Driver::generate_obstacle_(const std::vector<point2d_t<int>>& convex_hull)
{
    double res = multilayer_map_.getResolution();
    if (convex_hull.size() == 1)
    {
        grid_map::Index id(convex_hull[0].x, convex_hull[0].y);
        grid_map::Position position;
        multilayer_map_.getPosition(id, position);
        //circle ver
        return obstacle_t(position(0), position(1), sqrt(2) * res / 2);
    }

    double center_x, center_y, ra, rb, theta;
    if (convex_hull.size() == 2)
    {
        grid_map::Index id1(convex_hull[0].x, convex_hull[0].y);
        grid_map::Position position1;
        multilayer_map_.getPosition(id1, position1);
        grid_map::Index id2(convex_hull[1].x, convex_hull[1].y);
        grid_map::Position position2;
        multilayer_map_.getPosition(id2, position2);
        double dx = position1(0) - position2(0), dy = position1(1) - position2(1);
        center_x = (position1(0) + position2(0)) / 2;
        center_y = (position1(1) + position2(1)) / 2;
        ra = (sqrt(dx * dx + dy * dy) + res) / 2;
        rb = res / 2;
        theta = std::atan2(dy, dx);
        return obstacle_t(center_x, center_y, sqrt(ra*ra + rb*rb));
    }
    else
    {
        //find the minimum bounding box
        std::vector<point2d_t<double>> hull_world;
        for (int i = 0; i < convex_hull.size(); ++i)
        {
            grid_map::Index id(convex_hull[i].x, convex_hull[i].y);
            grid_map::Position position;
            multilayer_map_.getPosition(id, position);
            hull_world.push_back(point2d_t<double>(position(0), position(1)));
        }

        std::vector<double> circle = get_circle_bound(hull_world);
        return obstacle_t(circle[0], circle[1], circle[2] + res / 2);
    }
}

void Driver::updateSubgoal_()
{
    debugger::debugColorOutput("[updateSubgoal_] ",
                     " ", 5, C);
    click_point_lock_.lock();

    pose2d_t current_pose(inekf_state_.position, inekf_state_.orientation);

    if (!smooth_uref_)
    {
        if (subgoal1_selected_ && !Driver::reachedGoal_(current_pose, subgoal1_) && Driver::isSafe_(subgoal1_))
        {
            click_point_lock_.unlock();
            return;
        }
        subgoal1_selected_ = Driver::getSubgoal_(1.0, subgoal1_, current_pose);
    }
    else
    {
        bool new_subgoal1 = false;
        if (estimated_time_ == 0)
        {
            subgoal1_selected_ = false;
            if (subgoal2_selected_)
            {
                subgoal1_selected_ = true;
                subgoal2_selected_ = false;
                subgoal1_ = subgoal2_;
                new_subgoal1 = true;
            }
        }

        if (!subgoal1_selected_ || !Driver::isSafe_(subgoal1_))
        {
            subgoal1_selected_ = Driver::getSubgoal_(0.5, subgoal1_, current_pose);
            if (subgoal1_selected_)
            {
                new_subgoal1 = true;
            }
        }

        if (!subgoal2_selected_ || !Driver::isSafe_(subgoal2_))
        {
            subgoal2_selected_ = Driver::getSubgoal_(1.0, subgoal2_, current_pose);
        }

        if (!subgoal1_selected_ && subgoal2_selected_)
        {
            subgoal1_selected_ = true;
            subgoal2_selected_ = false;
            subgoal1_ = subgoal2_;
            new_subgoal1 = true;
        }

        if (new_subgoal1)
        {
            point2d_t<double> robot_pose(current_pose.x, current_pose.y);
            double dis = distance_pts(robot_pose, subgoal1_);
            estimated_time_ = dis / robot_param_.vx_upper_bound;
        }
    }
    click_point_lock_.unlock();
}

bool Driver::getSubgoal_(double scale, point2d_t<double>& subgoal, const pose2d_t& current_pose)
{
    bool subgoal_selected = false;

    //current goal reached, go to the next one
    if (Driver::reachedGoal_(current_pose, goal_list_[current_goal_id_]))
    {
        ++current_goal_id_;
    }

    if (goal_list_.size() > current_goal_id_)
    {
        point2d_t<double> current_goal = goal_list_[current_goal_id_];
        double dx = current_goal.x - current_pose.x;
        double dy = current_goal.y - current_pose.y;
        //if not in map, choose the intersection
        grid_map::Position position(current_goal.x, current_goal.y);
        if (!multilayer_map_.isInside(position))
        {
            double u = std::max(std::abs(dx), std::abs(dy));
            grid_map::Length map_len = multilayer_map_.getLength();
            double l = std::min(map_len(0) / 2, map_len(1) / 2) * scale;
            current_goal.x = current_pose.x + dx / u * l;
            current_goal.y = current_pose.y + dy / u * l;
        }
        else
        {
            current_goal.x = current_goal.x * scale + current_pose.x * (1 - scale);
            current_goal.y = current_goal.y * scale + current_pose.y * (1 - scale);
        }
        subgoal_selected = true;

        //if occupied, move goal back
        double l = sqrt(dx * dx + dy * dy);
        double step_x = dx / l * step_length_, step_y = dy / l * step_length_;
        while (!Driver::isSafe_(current_goal) && l > 0)
        {
            current_goal.x = current_goal.x - step_x;
            current_goal.y = current_goal.y - step_y;
            if (l > step_length_)
            {
                l -= step_length_;
            }
            else
            {
                current_goal.x = current_pose.x;
                current_goal.y = current_pose.y;
                subgoal_selected = false;
                break;
            }
        }
        subgoal = current_goal;
    }
    else
    {
        subgoal_selected = false;
    }
    return subgoal_selected;
}

bool Driver::isSafe_(const point2d_t<double>& position)
{
    for (int i = 0; i < obstacles_.size(); ++i)
    {
        point2d_t<double> obs_pos(obstacles_[i].x, obstacles_[i].y);
        double dis = distance_pts(position, obs_pos);
        if (dis < obstacles_[i].r)
        {
            return false;
        }
    }
    grid_map::Size GridMapSize = multilayer_map_.getSize();
    double res = multilayer_map_.getResolution();
    grid_map::Position p(position.x, position.y);
    grid_map::Index id;
    multilayer_map_.getIndex(p, id);
    point2d_t<int> pos(id(0), id(1));
    std::vector<point2d_t<int>> neighbor;
    getNeighborPoints(pos, neighbor, 0, GridMapSize(0), 0, GridMapSize(1),
                      2, safe_distance_ / res);
    for (int k = 0; k < neighbor.size(); ++k)
    {
        grid_map::Index index(neighbor[k].x, neighbor[k].y);
        double height = multilayer_map_.at(std::string("elevation"), index);
        if (!std::isnan(height) && height > obstalce_param_.obstacle_threshold)
        {
            return false;
        }
    }
    return true;
}

bool Driver::reachedGoal_(const pose2d_t& current_pose, const point2d_t<double>& goal)
{
    point2d_t robot_position(current_pose.x, current_pose.y);
    double dis = distance_pts(robot_position, goal);
    return dis < reached_threshold_;
}

void Driver::publishControlCommand_()
{
    debugger::debugColorOutput("[publishControlCommand_] ",
                     " ", 5, C);
    ros::Rate reactive_planner_rate(reactive_planner_rate_);
    while(ros::ok())
    {
        planner_info_to_controller_t control_command;
        if (subgoal1_selected_)
        {
            //update target position and obstacles
            clfcbf_->setTarget1(subgoal1_.x, subgoal1_.y);
            if (subgoal2_selected_)
            {
                clfcbf_->setTarget2(subgoal2_.x, subgoal2_.y);
            }
            clfcbf_->setObstacles(obstacles_);
            if (smooth_uref_)
            {
                clfcbf_->setEstimatedTime(estimated_time_);
            }

            //update robot pose
            pose2d_t current_pose(inekf_state_.position, inekf_state_.orientation);

            //generate optimal control command
            Driver::generateOptimalControl_(current_pose, control_command);
            std::cout << "fjinished" << std::endl;
            if (smooth_uref_)
            {
                estimated_time_ = std::max(0.0d, estimated_time_ - time_interval_);
            }
        }
        else
        {
            //walk in place
            control_command.behavior = 0;
            control_command.velocity[0] = 0;
            control_command.velocity[1] = 0;
            control_command.velocity[2] = 0;
        }

        //publish to robot
        udp_->publishToRobotGivenInfoToRobotType(control_command);
        Driver::publishPlannerMsg_(control_command);
        Driver::moveRobot_(control_command);

        ++cnt_;
        if (cnt_ == 10)
        {
            //update map
            Driver::updateMultilayerMap_();

            //update obstacles
            Driver::updateObstacles_();

            //update subgoal
            Driver::updateSubgoal_();

            Driver::publishToRviz_();
            cnt_ = 0;

            if (trajectory_id_ + 1 < start_x_.size() &&
                inekf_state_.position.x != start_x_[trajectory_id_ + 1] &&
                goal_list_.size() <= current_goal_id_ &&
                !subgoal1_selected_ && !subgoal2_selected_)
            {
                tf2::Quaternion q;
                q.setRPY(0, 0, start_theta_[trajectory_id_ + 1]/180.0*M_PI);

                inekf_state_.header.frame_id = map_frame_;
                inekf_state_.position.x = start_x_[trajectory_id_ + 1];
                inekf_state_.position.y = start_y_[trajectory_id_ + 1];
                inekf_state_.position.z = 0;
                inekf_state_.orientation = tf2::toMsg(q);
            }
        }

        reactive_planner_rate.sleep();
    }
}

void Driver::moveRobot_(const planner_info_to_controller_t& control_command)
{
    pose2d_t robot_pose(inekf_state_.position, inekf_state_.orientation);

    double x = robot_pose.x, y = robot_pose.y, theta = robot_pose.theta;
    double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
    double vx = control_command.velocity[0], vy = control_command.velocity[1];
    double omega = control_command.velocity[2];

    double vx_world = vx * cos_theta - vy * sin_theta;
    double vy_world = vx * sin_theta + vy * cos_theta;

    x += vx_world * time_interval_;
    y += vy_world * time_interval_;
    theta += omega * time_interval_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    inekf_state_.header.frame_id = map_frame_;
    inekf_state_.position.x = x;
    inekf_state_.position.y = y;
    inekf_state_.position.z = 0;
    inekf_state_.orientation = tf2::toMsg(q);
}

void Driver::generateOptimalControl_(const pose2d_t& current_pose,
                             planner_info_to_controller_t& control_command)
{
    debugger::debugColorOutput("[generateOptimalControl_] ",
                     " ", 5, C);
    //transform into robot state: r, delta, theta
    Eigen::VectorXd cbf_state1 = clfcbf_->computeEgoPolarCoordinate(
          current_pose.x, current_pose.y, current_pose.theta, 1);
    Eigen::VectorXd cbf_state2;
    if (subgoal2_selected_)
    {
        cbf_state2 = clfcbf_->computeEgoPolarCoordinate(
              current_pose.x, current_pose.y, current_pose.theta, 2);
    }
    double r = cbf_state1(0), delta = cbf_state1(1), theta = cbf_state1(2);

    //check if reached
    if (r <= reached_threshold_)
    {
        //walk in place
        control_command.behavior = 0;
        control_command.velocity[0] = 0;
        control_command.velocity[1] = 0;
        control_command.velocity[2] = 0;
        return;
    }

    //check if target is in FOV
    double fov = M_PI / (2 * clf_cbf_param_.beta);
    if (delta > fov)
    {
        //rotate in place clockwise
        control_command.behavior = 1;
        control_command.velocity[0] = 0;
        control_command.velocity[1] = 0;
        control_command.velocity[2] = robot_param_.default_omega;
        return;
    }
    if (delta < -fov)
    {
        //rotate in place counter-clockwise
        control_command.behavior = 2;
        control_command.velocity[0] = 0;
        control_command.velocity[1] = 0;
        control_command.velocity[2] = -robot_param_.default_omega;
        return;
    }

    //generate clf-cbf optimal control command
    Eigen::VectorXd control, refcontrol;
    if (subgoal2_selected_)
    {
        refcontrol = clfcbf_->generateOptimalCLFControl(cbf_state1, cbf_state2);
        control = clfcbf_->generateOptimalCBFControl(cbf_state1, cbf_state2);
    }
    else
    {
        control = clfcbf_->generateOptimalCBFControl(cbf_state1);
    }

    //enlarge control command based on vx, vy upper bound
    double vx = control(0), vy = control(1), omega = -control(2);
    double k = 1.0;
    if (vx != 0 && vy == 0)
    {
        k = std::abs(robot_param_.vx_upper_bound / vx);
    }
    else if (vx == 0 && vy != 0)
    {
        k = std::abs(robot_param_.vy_upper_bound / vy);
    }
    else if (vx != 0 && vy != 0)
    {
        k = std::min(std::abs(robot_param_.vx_upper_bound / vx), std::abs(robot_param_.vy_upper_bound / vy));
    }
    k = std::min(k, enlarge_scale_);

    //use clf-cbf control command
    control_command.behavior = 3;
    control_command.velocity[0] = vx * k;
    control_command.velocity[1] = vy * k;
    control_command.velocity[2] = omega * k;
    return;
}

void Driver::publishPlannerMsg_(const planner_info_to_controller_t& control_command)
{
    planner_msgs::State state;

    state.header.stamp = ros::Time::now();
    state.header.frame_id = map_frame_;

    state.behavior = control_command.behavior;
    state.velocity.x = control_command.velocity[0];
    state.velocity.y = control_command.velocity[1];
    state.velocity.z = control_command.velocity[2];

    planner_command_pub_.publish(state);
    return;
}

void Driver::publishToRviz_()
{
    visualization_msgs::MarkerArray marker_array;

    //add robot pose marker
    pose3d_t current_pose(inekf_state_.position, inekf_state_.orientation);

    visualization_msgs::Marker robot_pose_marker;
    plotting::addMarkerWithPose(robot_pose_marker,
            visualization_msgs::Marker::ARROW,
            "robot_pose",
            0, 1, 0, 1, // color
            current_pose, // pose
            0, 1.0 // count, time
            );
    marker_array.markers.push_back(robot_pose_marker);

    if (subgoal1_selected_)
    {
        //add subgoal marker
        visualization_msgs::Marker subgoal_marker;
        plotting::addMarker(subgoal_marker,
                visualization_msgs::Marker::SPHERE,
                "subgoal1",
                0, 0, 1, 1, // color
                subgoal1_.x, subgoal1_.y, 0,
                0, 0, 0, 1, // orientation
                0, 0.5, 0.5, 0.5 //id, marker size
                );
        subgoal_marker.lifetime = ros::Duration(1);
        marker_array.markers.push_back(subgoal_marker);
    }

    if (subgoal2_selected_)
    {
        //add subgoal marker
        visualization_msgs::Marker subgoal_marker;
        plotting::addMarker(subgoal_marker,
                visualization_msgs::Marker::SPHERE,
                "subgoal2",
                1, 1, 0, 1, // color
                subgoal2_.x, subgoal2_.y, 0,
                0, 0, 0, 1, // orientation
                0, 0.5, 0.5, 0.5 //id, marker size
                );
        subgoal_marker.lifetime = ros::Duration(1);
        marker_array.markers.push_back(subgoal_marker);
    }

    int cnt = 0;
    for (int i = 0; i < obstacles_.size(); ++i)
    {
        visualization_msgs::Marker obstacle_marker;
        plotting::addMarker(obstacle_marker,
                visualization_msgs::Marker::SPHERE,
                "obstacle",
                1, 0, 0, 1, // color
                obstacles_[i].x, obstacles_[i].y, 0.8,
                0, 0, 0, 1, // orientation
                cnt++, obstacles_[i].r*2, obstacles_[i].r*2, 0.1 //id, marker size
                );
        obstacle_marker.lifetime = ros::Duration(1);
        marker_array.markers.push_back(obstacle_marker);
    }

    marker_array_pub_.publish(marker_array);

    //publish trajectory
    if (subgoal1_selected_)
    {
        geometry_msgs::Point point1;
        point1.x = current_pose.x;
        point1.y = current_pose.y;
        point1.z = current_pose.z + 0.1;
        trajectory_markers_.markers[trajectory_id_].points.push_back(point1);
    }
    for (int i = 0; i <= trajectory_id_; ++i)
    {
        trajectory_markers_.markers[i].header.stamp = ros::Time::now();
    }
    trajectory_pub_.publish(trajectory_markers_);

    target_pub_.publish(target_markers_);
}

bool Driver::updateMultilayerMap_()
{
    bool received = true;

    pose2d_t current_pose(inekf_state_.position, inekf_state_.orientation);

    map_lock_.lock();
    if (multilayer_map_updated_)
    {
        multilayer_map_ = new_multilayer_map_;
        double res = new_multilayer_map_.getResolution();
        grid_map::Position pos(std::floor(current_pose.x / res) * res, std::floor(current_pose.y / res) * res);
        grid_map::Length local_len(local_len_, local_len_);
        if (multilayer_map_.exists(std::string("elevation")))
        {
            multilayer_map_.erase(std::string("elevation"));
        }
        multilayer_map_.setGeometry(local_len, res, pos);
        multilayer_map_.add(std::string("elevation"), 0);
        auto& data = multilayer_map_["elevation"];
        for (grid_map::GridMapIterator it(multilayer_map_); !it.isPastEnd(); ++it)
        {
            const grid_map::Index id(*it);
            size_t i = it.getLinearIndex();
            grid_map::Position position;
            multilayer_map_.getPosition(id, position);
            if (new_multilayer_map_.isInside(position))
            {
                data(i) = new_multilayer_map_.atPosition(std::string("elevation"), position);
            }
        }
    }
    else
    {
        received = false;
        debugger::debugWarningOutput("[Driver]/[updateMultilayerMap] "
                "No new map: using the existing map", "", 10);
    }
    multilayer_map_updated_ = false;
    map_lock_.unlock();

    grid_map_msgs::GridMap local_map_msg;
    multilayer_map_.setTimestamp(ros::Time::now().toNSec());
    grid_map::GridMapRosConverter::toMessage(multilayer_map_, local_map_msg);
    local_map_pub_.publish(local_map_msg);

    return received;
}

void Driver::waitForData_() {
    if (goal_mode_ == 0)
    {
        while (ros::ok() &&
               (!multilayer_map_updated_ || !goal_received_))
        {
            ROS_WARN_THROTTLE(1, "Received map: %i", multilayer_map_updated_);
            ROS_WARN_THROTTLE(1, "Received goal: %i", goal_received_);
            sleep(0.5);
        }
    }
    return;
}

void Driver::getMultiLayerMapCallBack_(const grid_map_msgs::GridMap& grid_map_msg)
{
    map_lock_.lock();
    grid_map::GridMapRosConverter::fromMessage(grid_map_msg, new_multilayer_map_);
    multilayer_map_updated_ = true;
    map_lock_.unlock();
}

void Driver::getClickPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    click_point_lock_.lock();
    ++trajectory_id_;
    goal_list_.push_back(point2d_t(target_x_[trajectory_id_], target_y_[trajectory_id_]));

    //add goal marker
    visualization_msgs::Marker target_marker;
    plotting::addMarker(target_marker,
            visualization_msgs::Marker::SPHERE,
            "finalgoal",
            color_r_[color_id_], color_g_[color_id_], color_b_[color_id_], 1, // color
            target_x_[trajectory_id_], target_y_[trajectory_id_], 0,
            0, 0, 0, 1, // orientation
            trajectory_id_, 0.4, 0.4, 0.4 //id, marker size
            );
    target_markers_.markers.push_back(target_marker);

    //add start position marker
    visualization_msgs::Marker start_marker;
    plotting::addMarker(start_marker,
            visualization_msgs::Marker::CUBE,
            "startPosition",
            color_r_[color_id_], color_g_[color_id_], color_b_[color_id_], 1, // color
            start_x_[trajectory_id_], start_y_[trajectory_id_], 0,
            0, 0, 0, 1, // orientation
            trajectory_id_, 0.4, 0.4, 0.4 //id, marker size
            );
    target_markers_.markers.push_back(start_marker);

    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.frame_id = std::string("odom");
    trajectory_marker.header.stamp = ros::Time::now();
    trajectory_marker.ns = std::string("trajectory");
    trajectory_marker.id = trajectory_markers_.markers.size();
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.lifetime = ros::Duration(1.0 / subgoal_update_rate_); // should disappear along with updateing rate

    trajectory_marker.pose.orientation.x = 0;
    trajectory_marker.pose.orientation.y = 0;
    trajectory_marker.pose.orientation.z = 0;
    trajectory_marker.pose.orientation.w = 1;

    trajectory_marker.scale.x = 0.1;
    trajectory_marker.scale.y = 0.1;
    trajectory_marker.scale.z = 0.01;

    trajectory_marker.color.r = color_r_[color_id_];
    trajectory_marker.color.g = color_g_[color_id_];
    trajectory_marker.color.b = color_b_[color_id_];
    trajectory_marker.color.a = 1;

    trajectory_markers_.markers.push_back(trajectory_marker);
    color_id_ = (color_id_ + 1) % color_r_.size();

    goal_received_ = true;
    click_point_lock_.unlock();
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParameters] ");
    bool received_all = true;

    ros_utils::checkROSParam(nh_, "goal_mode", goal_mode_,
        getNameOf(goal_mode_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "multilayer_map_topic", multilayer_map_topic_,
        getNameOf(multilayer_map_topic_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "map_frame", map_frame_,
        getNameOf(map_frame_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "local_len", local_len_,
            getNameOf(local_len_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "reached_threshold", reached_threshold_,
        getNameOf(reached_threshold_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "step_length", step_length_,
        getNameOf(step_length_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "safe_distance", safe_distance_,
        getNameOf(safe_distance_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "goal_distance", goal_distance_,
        getNameOf(goal_distance_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "subgoal_update_rate", subgoal_update_rate_,
            getNameOf(subgoal_update_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "reactive_planner_rate", reactive_planner_rate_,
            getNameOf(reactive_planner_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "time_interval", time_interval_,
            getNameOf(time_interval_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "enlarge_scale", enlarge_scale_,
            getNameOf(enlarge_scale_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "smooth_uref", smooth_uref_,
            getNameOf(smooth_uref_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "simulation/start_x", start_x_, getNameOf(start_x_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "simulation/start_y", start_y_, getNameOf(start_y_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "simulation/start_theta", start_theta_, getNameOf(start_theta_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "simulation/target_x", target_x_, getNameOf(target_x_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "simulation/target_y", target_y_, getNameOf(target_y_), title_name, received_all);

    tf2::Quaternion q;
    q.setRPY(0, 0, start_theta_[0]/180.0*M_PI);

    inekf_state_.header.frame_id = map_frame_;
    inekf_state_.position.x = start_x_[0];
    inekf_state_.position.y = start_y_[0];
    inekf_state_.position.z = 0;
    inekf_state_.orientation = tf2::toMsg(q);

    //robot parameters
    ros_utils::checkROSParam(nh_, "robot_param/vx_upper_bound",
            robot_param_.vx_upper_bound,
            getNameOf(robot_param_) + "." +
            getNameOf(robot_param_.vx_upper_bound),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "robot_param/vy_upper_bound",
            robot_param_.vy_upper_bound,
            getNameOf(robot_param_) + "." +
            getNameOf(robot_param_.vy_upper_bound),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "robot_param/default_omega",
            robot_param_.default_omega,
            getNameOf(robot_param_) + "." +
            getNameOf(robot_param_.default_omega),
            title_name, received_all);

    robot_param_.default_omega = robot_param_.default_omega / 180.0 * M_PI;

    //clf-cbf parameters
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

    ros_utils::checkROSParam(nh_, "clf_cbf/eps",
            clf_cbf_param_.eps,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.eps),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/delta_dis",
            clf_cbf_param_.delta_dis,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.delta_dis),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/switch_time",
            clf_cbf_param_.switch_time,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.switch_time),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/max_ob",
            clf_cbf_param_.max_ob,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.max_ob),
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

    ros_utils::checkROSParam(nh_, "clf_cbf/h1",
            clf_cbf_param_.h1,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.h1),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/h2",
            clf_cbf_param_.h2,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.h2),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "clf_cbf/h3",
            clf_cbf_param_.h3,
            getNameOf(clf_cbf_param_) + "." +
            getNameOf(clf_cbf_param_.h3),
            title_name, received_all);

    // communication
    ros_utils::checkROSParam(nh_, "communication/port_to_send",
            udp_info_.port_to_send,
            getNameOf(udp_info_) + "." + getNameOf(udp_info_.port_to_send),
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "communication/port_to_receive",
            udp_info_.port_to_receive,
            getNameOf(udp_info_) + "." + getNameOf(udp_info_.port_to_receive),
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "communication/ip_to_send", udp_info_.ip_to_send,
            getNameOf(udp_info_) + "." + getNameOf(udp_info_.ip_to_send),
            title_name, received_all);

    //obstacle detection parameter
    ros_utils::checkROSParam(nh_, "obstacles/obstacle_threshold",
        obstalce_param_.obstacle_threshold,
        getNameOf(obstalce_param_) + "." +
        getNameOf(obstalce_param_.obstacle_threshold),
        title_name, received_all);

    ros_utils::checkROSParam(nh_, "obstacles/narrow_distance",
        obstalce_param_.narrow_distance,
        getNameOf(obstalce_param_) + "." +
        getNameOf(obstalce_param_.narrow_distance),
        title_name, received_all);

    //color
    ros_utils::checkROSParam(nh_, "color/r", color_r_,
        getNameOf(color_r_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "color/g", color_g_,
        getNameOf(color_g_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "color/b", color_b_,
        getNameOf(color_b_), title_name, received_all);

    return received_all;
}

Driver::~Driver() { }

}
