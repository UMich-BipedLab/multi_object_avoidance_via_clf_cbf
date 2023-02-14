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
 * File:        fake_map.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: Publish fake map with circle obstacles
*******************************************************************************/
#include "fake_map.h"

namespace bipedlab
{

fake_map::fake_map(ros::NodeHandle& nh): nh_(nh)
{
    if (!fake_map::getParameters_())
    {
        debugger::debugTitleTextOutput("[fake_map]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[fake_map] Not enough parameters: ",
                                    "Using default values", 10, BR, BOLD);
        debugger::debugTitleTextOutput("[fake_map]", "", 10, BR, BOLD);
        utils::pressEnterToContinue();
    }
    else
    {
        debugger::debugColorOutput("[fake_map] Received all parameters", "", 10, BC);
    }

    //publisher
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap> ("elevation_map", 1);

    //load fake map
    if (map_id_ == 0)
    {
        fake_map::loadMap_();
    }
    else
    {
        fake_map::loadHeightMap_();
    }

    if (add_nan_noise_)
    {
        fake_map::addNanNoise_();
    }

    //publish fake map
    fake_map::publishMap_();
}

void fake_map::loadMap_()
{
    elevation_map_.setFrameId(map_frame_);
    grid_map::Length map_length(map_info_.length_x, map_info_.length_y);
		grid_map::Position center_position(map_info_.center_x, map_info_.center_y);
    elevation_map_.setGeometry(map_length, map_info_.resolution, center_position);
    elevation_map_.add(std::string("elevation"), 0);

    const grid_map::Size&  GridMapSize = elevation_map_.getSize();

    for (int i = 0; i < obstacles_.size(); ++i)
    {
        grid_map::Position pos(obstacles_[i].x, obstacles_[i].y);
        grid_map::Index id;
        if (elevation_map_.getIndex(pos, id))
        {
            double len = obstacles_[i].r / map_info_.resolution;
            for (int x = std::max(0, id(0) - (int)std::floor(len));
                      x <= std::min(GridMapSize(0) - 1, id(0) + (int)std::floor(len)); ++x)
            {
                int dx = abs(x - id(0));
                int len_y = (int)std::floor(sqrt(len * len - dx * dx));
                for (int y = std::max(0, id(1) - len_y);
                      y <= std::min(GridMapSize(1) - 1, id(1) + len_y); ++y)
                {
                    grid_map::Index index(x, y);
                    elevation_map_.at("elevation", index) = 1;
                }
            }
        }
    }
}

void fake_map::loadHeightMap_()
{
    elevation_map_.setFrameId(map_frame_);
    grid_map::Length map_length(map_info_.length_x, map_info_.length_y);
		grid_map::Position center_position(map_info_.center_x, map_info_.center_y);
    elevation_map_.setGeometry(map_length, map_info_.resolution, center_position);
    elevation_map_.add(std::string("elevation"), 0);

    const grid_map::Size&  GridMapSize = elevation_map_.getSize();

    for (int i = 0; i < obstacles_.size(); ++i)
    {
        grid_map::Position pos(obstacles_[i].x, obstacles_[i].y);
        grid_map::Index id;
        if (elevation_map_.getIndex(pos, id))
        {
            double len = obstacles_[i].r / map_info_.resolution;
            for (int x = std::max(0, id(0) - (int)std::floor(len));
                      x <= std::min(GridMapSize(0) - 1, id(0) + (int)std::floor(len)); ++x)
            {
                int dx = abs(x - id(0));
                int len_y = (int)std::floor(sqrt(len * len - dx * dx));
                for (int y = std::max(0, id(1) - len_y);
                      y <= std::min(GridMapSize(1) - 1, id(1) + len_y); ++y)
                {
                    int dy = abs(y - id(1));
                    double dis = sqrt(dx * dx + dy * dy) * map_info_.resolution;
                    grid_map::Index index(x, y);
                    double height = obs_height_[i] * exp(-dis * dis / (2*obs_std_[i]*obs_std_[i]));
                    double original = elevation_map_.at("elevation", index);
                    elevation_map_.at("elevation", index) = std::max(height, original);
                }
            }
        }
    }
}

void fake_map::addNanNoise_()
{
    grid_map::Size GridMapSize = elevation_map_.getSize();
    std::srand(std::time(nullptr));
    for (int i = 0; i < 1600; ++i)
    {
        int x = rand() % GridMapSize(0);
        int y = rand() % GridMapSize(1);
        int s = rand() % 6;
        grid_map::Index index(x, y);
        elevation_map_.at("elevation", index) = NAN;
        if (s == 1 || s >= 3 )
        {
            grid_map::Index id(x + 1, y);
            if (elevation_map_.isValid(id))
            {
                elevation_map_.at("elevation", id) = NAN;
            }
        }
        if (s == 2 || s >= 3)
        {
            grid_map::Index id(x, y + 1);
            if (elevation_map_.isValid(id))
            {
                elevation_map_.at("elevation", id) = NAN;
            }
        }
        if (s >= 3)
        {
            grid_map::Index id(x + 1, y + 1);
            if (elevation_map_.isValid(id))
            {
                elevation_map_.at("elevation", id) = NAN;
            }
        }
        if (s == 4)
        {
            grid_map::Index id1(x + 2, y);
            if (elevation_map_.isValid(id1))
            {
                elevation_map_.at("elevation", id1) = NAN;
            }
            grid_map::Index id2(x + 2, y + 1);
            if (elevation_map_.isValid(id2))
            {
                elevation_map_.at("elevation", id2) = NAN;
            }
        }
        if (s == 5)
        {
            grid_map::Index id1(x, y + 2);
            if (elevation_map_.isValid(id1))
            {
                elevation_map_.at("elevation", id1) = NAN;
            }
            grid_map::Index id2(x, y + 2);
            if (elevation_map_.isValid(id2))
            {
                elevation_map_.at("elevation", id2) = NAN;
            }
        }
    }
}

void fake_map::publishMap_()
{
    grid_map::GridMapRosConverter::toMessage(elevation_map_, fake_map_msg_);
    ros::Rate publish_rate(publish_rate_);
    while (ros::ok())
    {
        fake_map_msg_.info.header.stamp = ros::Time::now();
        map_pub_.publish(fake_map_msg_);
        publish_rate.sleep();
    }
}

bool fake_map::getParameters_()
{
    std::string title_name("[fake_map]/[getParameters] ");
    bool received_all = true;

    ros_utils::checkROSParam(nh_, "fake_map/publish_rate", publish_rate_,
            getNameOf(publish_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/map_id", map_id_,
            getNameOf(map_id_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/map_frame", map_frame_,
        getNameOf(map_frame_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/add_nan_noise", add_nan_noise_,
        getNameOf(add_nan_noise_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/length_x",
            map_info_.length_x,
            getNameOf(map_info_) + "." +
            getNameOf(map_info_.length_x),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/length_y",
            map_info_.length_y,
            getNameOf(map_info_) + "." +
            getNameOf(map_info_.length_y),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/center_x",
            map_info_.center_x,
            getNameOf(map_info_) + "." +
            getNameOf(map_info_.center_x),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/center_y",
            map_info_.center_y,
            getNameOf(map_info_) + "." +
            getNameOf(map_info_.center_y),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_map/resolution",
            map_info_.resolution,
            getNameOf(map_info_) + "." +
            getNameOf(map_info_.resolution),
            title_name, received_all);

    std::vector<double> center_x;
    ros_utils::checkROSParam(nh_, "obstacles/center_x", center_x,
        getNameOf(center_x), title_name, received_all);

    std::vector<double> center_y;
    ros_utils::checkROSParam(nh_, "obstacles/center_y", center_y,
        getNameOf(center_y), title_name, received_all);

    std::vector<double> radius;
    ros_utils::checkROSParam(nh_, "obstacles/radius", radius,
        getNameOf(radius), title_name, received_all);

    int n = std::min(std::min(center_x.size(), center_y.size()), radius.size());
    for (int i = 0; i < n; ++i)
    {
        obstacles_.push_back(obstacle_t(center_x[i], center_y[i], radius[i]));
    }

    if (map_id_ != 0)
    {
        ros_utils::checkROSParam(nh_, "obstacles/height", obs_height_,
            getNameOf(obs_height_), title_name, received_all);

        ros_utils::checkROSParam(nh_, "obstacles/std", obs_std_,
            getNameOf(obs_std_), title_name, received_all);
    }

    return received_all;
}

fake_map::~fake_map() { }

}
