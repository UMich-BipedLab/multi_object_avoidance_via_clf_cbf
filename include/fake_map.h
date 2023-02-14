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
 * File:        fake_map.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: Publish fake map with circle obstacles
*******************************************************************************/
#ifndef FAKE_MAP_H
#define FAKE_MAP_H

#include "utils/utils.h"
#include "utils/ros_utils.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cstdlib>
#include <iostream>
#include <ctime>

#include <vector>

namespace bipedlab
{

struct map_info_t
{
    double length_x, length_y;
    double center_x, center_y;
    double resolution;
};

struct obstacle_t
{
    double x;
    double y;
    double r;

    obstacle_t(double x_, double y_, double r_): x(x_), y(y_), r(r_) { }
};

class fake_map
{
private:
    //initialization functions
    bool getParameters_();

    //load occupancy grid map
    void loadMap_();

    //load elevation grid map
    void loadHeightMap_();

    //publish elevation map
    void publishMap_();

    //add random nan holes to the map
    void addNanNoise_();

    // Map
    grid_map::GridMap elevation_map_;
    grid_map_msgs::GridMap fake_map_msg_;

    // obstacles
    std::vector<obstacle_t> obstacles_;
    std::vector<double> obs_height_, obs_std_;

    // ROS
    ros::NodeHandle nh_;

    //publisher
    ros::Publisher map_pub_;

    //frames
    std::string map_frame_;

    //publish rate
    double publish_rate_;

    //parameter
    map_info_t map_info_;
    int map_id_;
    bool add_nan_noise_;

public:

    fake_map(ros::NodeHandle& nh);
    virtual ~fake_map();

};

} /*  bipedlab */
#endif /* FAKE_MAP_H */
