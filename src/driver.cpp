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
 * File:        driver.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: ROS driver used in autonomy experiment
*******************************************************************************/
#include "driver.h"

namespace bipedlab
{

Driver::Driver(ros::NodeHandle& nh): nh_(nh),
multilayer_map_updated_(false), inekf_updated_(false), goal_received_(false),
subgoal1_selected_(false), subgoal2_selected_(false), current_goal_id_(0),
time_initialized_(false), estimated_time_(0)
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
    inekf_sub_ = nh_.subscribe(inekf_topic_, 10,
                                &Driver::getInEKFCallBack_, this);
    map_sub_ = nh_.subscribe(multilayer_map_topic_, 10,
                                &Driver::getMultiLayerMapCallBack_, this);
    click_point_sub_ = nh_.subscribe(std::string("/clicked_point"), 10,
                                &Driver::getClickPointCallBack_, this);

    //publisher
    planner_command_pub_ = nh_.advertise<planner_msgs::State> (
            "planner_commands", 1);
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "MarkerArray", 10);

    //spinner
    ros::AsyncSpinner spinner(nh_.param("num_callback_threads", 4));
    spinner.start();

    // initialize classes
    udp_ = new Communication(udp_info_);
    clfcbf_ = new clf_cbf(clf_cbf_param_);

    // wait for data arriving
    Driver::waitForData_();

    //update inekf state and multilayer map
    Driver::updateInEKF_();
    Driver::updateMultilayerMap_();

    //execute clf-cbf and publish control command to robot
    boost::thread publishControlCommand(&Driver::publishControlCommand_, this);

    //drive robot
    Driver::driveRobot_();
}

void Driver::driveRobot_()
{
    ros::Rate subgoal_update_rate(subgoal_update_rate_);
    while(ros::ok())
    {
        //update robot pose
        Driver::updateInEKF_(subgoal_inekf_);
        pose2d_t current_pose(subgoal_inekf_.position, subgoal_inekf_.orientation);

        debugger::debugColorOutput("[Driver Robot] =======================",
                     current_pose.to_string(), 5, C);

        //update map
        Driver::updateMultilayerMap_();

        //update obstacles
        Driver::updateObstacles_();

        //update subgoal
        Driver::updateSubgoal_(current_pose);

        debugger::debugColorOutput("[Driver Robot] subgoal1: ",
                     subgoal1_.to_string(), 5, C);
        debugger::debugColorOutput("[Driver Robot] subgoal2: ",
                     subgoal2_.to_string(), 5, C);

        //publish to rviz
        Driver::publishToRviz_();
        subgoal_update_rate.sleep();
    }
}

void Driver::updateObstacles_()
{
    debugger::debugColorOutput("[updateObstacles_] ",
                     " ", 5, C);
    std::vector<obstacle_t> new_obstacles;
    std::vector<std::vector<point2d_t<int>>> convex_hulls;
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

    //Find obstacles
    std::vector<std::vector<bool>> visited(GridMapSize(0),
                              std::vector<bool>(GridMapSize(1), false));
    int obstacle_n = 0;
    for (int i = 0; i < GridMapSize(0); ++i)
    {
        for (int j = 0; j < GridMapSize(1); ++j)
        {
            if (occupancy[i][j] == 1 && !visited[i][j])
            {
                //generate boundary points
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

                //find convex hull
                std::vector<point2d_t<int>> convex_hull;
                if (bound_points.size() == 1)
                {
                    convex_hull = bound_points;
                }
                else
                {
                    convex_hull = get_convex_hull(bound_points);
                }

                //generate obstacle
                obstacle_t new_obstacle = Driver::generate_obstacle_(convex_hull);
                if (new_obstacle.r < 2)
                {
                    new_obstacles.push_back(new_obstacle);
                    convex_hulls.push_back(convex_hull);
                }
            }
        }
    }

    obstacle_lock_.lock();
    obstacles_.clear();
    obstacles_ = new_obstacles;
    obstacle_lock_.unlock();
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

void Driver::updateSubgoal_(const pose2d_t& current_pose)
{
    debugger::debugColorOutput("[updateSubgoal_] ",
                     " ", 5, C);

    if (!smooth_uref_)
    {
        if (subgoal1_selected_ && !Driver::reachedGoal_(current_pose, subgoal1_) && Driver::isSafe_(subgoal1_))
        {
            debugger::debugColorOutput("[updateSubgoal_] No reached!",
                                        " ", 5, C);
            return;
        }

        point2d_t<double> subgoal1 = subgoal1_;
        bool subgoal1_selected = subgoal1_selected_;

        click_point_lock_.lock();
        subgoal1_selected = Driver::getSubgoal_(1.0, subgoal1, current_pose);
        click_point_lock_.unlock();

        subgoal_lock_.lock();
        subgoal1_selected_ = subgoal1_selected;
        subgoal1_ = subgoal1;
        subgoal_lock_.unlock();
    }
    else
    {
        bool new_subgoal1 = false;
        point2d_t<double> subgoal1 = subgoal1_, subgoal2 = subgoal2_;
        bool subgoal1_selected = subgoal1_selected_, subgoal2_selected = subgoal2_selected_;
        if (estimated_time_ == 0)
        {
            subgoal1_selected = false;
            if (subgoal2_selected)
            {
                subgoal1_selected = true;
                subgoal2_selected = false;
                subgoal1 = subgoal2;
                new_subgoal1 = true;
            }
        }

        click_point_lock_.lock();
        if (!subgoal1_selected || !Driver::isSafe_(subgoal1))
        {
            subgoal1_selected = Driver::getSubgoal_(0.5, subgoal1, current_pose);
            if (subgoal1_selected)
            {
                new_subgoal1 = true;
            }
        }
        if (!subgoal2_selected || !Driver::isSafe_(subgoal2))
        {
            subgoal2_selected = Driver::getSubgoal_(1.0, subgoal2, current_pose);
        }
        click_point_lock_.unlock();

        if (!subgoal1_selected && subgoal2_selected)
        {
            subgoal1_selected = true;
            subgoal2_selected = false;
            subgoal1 = subgoal2;
            new_subgoal1 = true;
        }

        subgoal_lock_.lock();
        if (new_subgoal1)
        {
            point2d_t<double> robot_pose(current_pose.x, current_pose.y);
            double dis = distance_pts(robot_pose, subgoal1_);
            estimated_time_ = dis / robot_param_.vx_upper_bound;
        }

        subgoal1_selected_ = subgoal1_selected;
        subgoal2_selected_ = subgoal2_selected;
        subgoal1_ = subgoal1;
        subgoal2_ = subgoal2;
        subgoal_lock_.unlock();
    }
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
    ros::Rate reactive_planner_rate(reactive_planner_rate_);
    while(ros::ok())
    {
        planner_info_to_controller_t control_command;
        obstacle_lock_.lock();
        subgoal_lock_.lock();

        //update robot pose
        inekf_msgs::State current_state;
        Driver::updateInEKF_(current_state);
        pose2d_t current_pose(current_state.position, current_state.orientation);
        double time_interval = 0;
        if (!time_initialized_)
        {
            last_time_ = current_state.header.stamp;
            time_initialized_ = true;
        }
        else
        {
            ros::Duration interval = current_state.header.stamp - last_time_;
            time_interval = interval.toSec();
            last_time_ = current_state.header.stamp;
        }

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
            subgoal_lock_.unlock();
            obstacle_lock_.unlock();

            //generate optimal control command
            Driver::generateOptimalControl_(current_pose, control_command);

            if (smooth_uref_)
            {
                estimated_time_ = std::max(0.0d, estimated_time_ - time_interval);
            }
        }
        else
        {
            subgoal_lock_.unlock();
            obstacle_lock_.unlock();

            //walk in place
            control_command.behavior = 0;
            control_command.velocity[0] = 0;
            control_command.velocity[1] = 0;
            control_command.velocity[2] = 0;
            control_command.torso.yaw = current_pose.theta;
        }

        //publish to robot
        udp_->publishToRobotGivenInfoToRobotType(control_command);
        Driver::publishPlannerMsg_(control_command);

        reactive_planner_rate.sleep();
    }
}

void Driver::generateOptimalControl_(const pose2d_t& current_pose,
                             planner_info_to_controller_t& control_command)
{
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
        control_command.torso.yaw = current_pose.theta;
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
        control_command.torso.yaw = current_pose.theta +
            robot_param_.default_omega * robot_param_.step_time_interval;
        return;
    }
    if (delta < -fov)
    {
        //rotate in place counter-clockwise
        control_command.behavior = 2;
        control_command.velocity[0] = 0;
        control_command.velocity[1] = 0;
        control_command.velocity[2] = -robot_param_.default_omega;
        control_command.torso.yaw = current_pose.theta -
            robot_param_.default_omega * robot_param_.step_time_interval;
        return;
    }

    //generate clf-cbf optimal control command
    Eigen::VectorXd control;
    if (subgoal2_selected_)
    {
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
    control_command.torso.yaw = current_pose.theta +
        omega * k * robot_param_.step_time_interval;
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
    state.torso.yaw = control_command.torso.yaw;

    planner_command_pub_.publish(state);
    return;
}

void Driver::publishToRviz_()
{
    visualization_msgs::MarkerArray marker_array;

    //add robot pose marker
    int cnt = 0;
    pose3d_t current_pose(subgoal_inekf_.position, subgoal_inekf_.orientation);

    visualization_msgs::Marker robot_pose_marker;
    plotting::addMarkerWithPose(robot_pose_marker,
            visualization_msgs::Marker::ARROW,
            "robot_pose",
            0, 1, 0, 1, // color
            current_pose, // pose
            cnt++, 1.0 // count, time
            );
    marker_array.markers.push_back(robot_pose_marker);

    //add subgoal marker
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

    //add obstacle
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
        marker_array.markers.push_back(obstacle_marker);
    }

    //publish trajectory
    if (trajectory_marker_.points.size() == 0)
    {
        //initialize
        trajectory_marker_.header.frame_id = std::string("odom");
      	trajectory_marker_.header.stamp = ros::Time::now();
      	trajectory_marker_.ns = std::string("trajectory");
      	trajectory_marker_.id = 0;
      	trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
      	trajectory_marker_.lifetime = ros::Duration(1.0 / subgoal_update_rate_); // should disappear along with updateing rate

      	trajectory_marker_.pose.orientation.x = 0;
      	trajectory_marker_.pose.orientation.y = 0;
      	trajectory_marker_.pose.orientation.z = 0;
      	trajectory_marker_.pose.orientation.w = 1;

      	// Set the scale of the marker -- 1x1x1 here means 1m on a side
      	trajectory_marker_.scale.x = 0.1;
      	trajectory_marker_.scale.y = 0.01;
      	trajectory_marker_.scale.z = 0.01;

      	// Set the color -- be sure to set alpha to something non-zero!
      	trajectory_marker_.color.r = 0;
      	trajectory_marker_.color.g = 1;
      	trajectory_marker_.color.b = 0;
      	trajectory_marker_.color.a = 1;
    }
    geometry_msgs::Point point1;
    point1.x = current_pose.x;
    point1.y = current_pose.y;
    point1.z = current_pose.z;
    trajectory_marker_.points.push_back(point1);
    trajectory_marker_.header.stamp = ros::Time::now();
    marker_array.markers.push_back(trajectory_marker_);

    marker_array_pub_.publish(marker_array);
}

bool Driver::updateInEKF_()
{
    inekf_msgs::State current_state;
    return Driver::updateInEKF_(current_state);
}

bool Driver::updateInEKF_(inekf_msgs::State& current_state)
{
    bool received = true;

    inekf_lock_.lock();
    if (inekf_updated_)
    {
        inekf_state_.header = new_inekf_state_.header;
        inekf_state_.orientation = new_inekf_state_.orientation;
        inekf_state_.position = new_inekf_state_.position;
        inekf_state_.velocity = new_inekf_state_.velocity;
    }
    else
    {
        received = false;
    }
    inekf_updated_ = false;
    current_state.orientation = inekf_state_.orientation;
    current_state.position = inekf_state_.position;
    inekf_lock_.unlock();

    return received;
}

bool Driver::updateMultilayerMap_()
{
    bool received = true;
    debugger::debugColorOutput("[updateMultilayerMap_] ",
                     multilayer_map_updated_, 5, C);
    map_lock_.lock();
    if (multilayer_map_updated_)
    {
        multilayer_map_ = new_multilayer_map_;
    }
    else
    {
        received = false;
    }
    multilayer_map_updated_ = false;
    map_lock_.unlock();

    return received;
}

void Driver::waitForData_()
{
    while (ros::ok() &&
           (!multilayer_map_updated_ || !inekf_updated_ || !goal_received_))
    {
        ROS_WARN_THROTTLE(1, "Received map: %i", multilayer_map_updated_);
        ROS_WARN_THROTTLE(1, "Received pose: %i", inekf_updated_);
        ROS_WARN_THROTTLE(1, "Received goal: %i", goal_received_);
        sleep(0.5);
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

void Driver::getInEKFCallBack_(const inekf_msgs::State& msg)
{
    inekf_lock_.lock();
    new_inekf_state_.header = msg.header;
    new_inekf_state_.orientation = msg.orientation;
    new_inekf_state_.position = msg.position;
    new_inekf_state_.velocity = msg.velocity;
    inekf_updated_ = true;
    inekf_lock_.unlock();
}

void Driver::getClickPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    click_point_lock_.lock();
    goal_list_.push_back(point2d_t(msg->point.x, msg->point.y));
    goal_received_ = true;
    click_point_lock_.unlock();
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParameters] ");
    bool received_all = true;

    ros_utils::checkROSParam(nh_, "multilayer_map_topic", multilayer_map_topic_,
        getNameOf(multilayer_map_topic_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "inekf_topic", inekf_topic_,
        getNameOf(inekf_topic_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "map_frame", map_frame_,
        getNameOf(map_frame_), title_name, received_all);

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

    ros_utils::checkROSParam(nh_, "enlarge_scale", enlarge_scale_,
            getNameOf(enlarge_scale_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "smooth_uref", smooth_uref_,
            getNameOf(smooth_uref_), title_name, received_all);

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

    ros_utils::checkROSParam(nh_, "robot_param/step_time_interval",
            robot_param_.step_time_interval,
            getNameOf(robot_param_) + "." +
            getNameOf(robot_param_.step_time_interval),
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

    return received_all;
}

Driver::~Driver() { }

}
