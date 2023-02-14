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
 * File:        pose.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: 2d pose struct and 3d pose struct
*******************************************************************************/
#ifndef POSE_H
#define POSE_H

#include <cmath>
#include <string>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

namespace bipedlab
{

struct pose2d_t
{
    double x;
    double y;
    double theta;

    pose2d_t(void) : x(0), y(0), theta(0) { }

    pose2d_t(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) { }

    pose2d_t(geometry_msgs::Point p, geometry_msgs::Quaternion q): x(p.x), y(p.y)
    {
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(tf_q);
        double roll_, pitch_, yaw_;
        m.getRPY(roll_, pitch_, yaw_);
        theta = yaw_;
    }

    std::string to_string()
    {
        return std::string("(") + std::to_string(x) + std::string(", ") +
               std::to_string(y) + std::string(", ") + std::to_string(theta) +
               std::string(")");
    }

    inline bool operator== (const pose2d_t& rhs) const
    {
        return (std::abs(x - rhs.x) < 0.0001) &&
               (std::abs(y - rhs.y) < 0.0001) &&
               (std::abs(theta - rhs.theta) < 0.0001);
    }

    inline bool operator!= (const pose2d_t& rhs) const
    {
        return (std::abs(x - rhs.x) >= 0.0001) ||
               (std::abs(y - rhs.y) >= 0.0001) ||
               (std::abs(theta - rhs.theta) >= 0.0001);
    }

    inline pose2d_t& operator= (const pose2d_t& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        theta = rhs.theta;
        return *this;
    }

};

struct pose3d_t
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    pose3d_t(void) : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) { }

    pose3d_t(double x_, double y_, double z_,
             double roll_, double pitch_, double yaw_) :
             x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_) { }

    pose3d_t(geometry_msgs::Point p, geometry_msgs::Quaternion q):
              x(p.x), y(p.y), z(p.z)
    {
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(tf_q);
        double roll_, pitch_, yaw_;
        m.getRPY(roll_, pitch_, yaw_);
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
    }

    std::string to_string()
    {
        return std::string("(") + std::to_string(x) + std::string(", ") +
               std::to_string(y) + std::string(", ") + std::to_string(z) +
               std::string(", ") + std::to_string(roll) + std::string(", ") +
               std::to_string(pitch) + std::string(", ") + std::to_string(yaw) +
               std::string(")");
    }

    inline bool operator== (const pose3d_t& rhs) const
    {
        return (std::abs(x - rhs.x) < 0.0001) &&
               (std::abs(y - rhs.y) < 0.0001) &&
               (std::abs(z - rhs.z) < 0.0001) &&
               (std::abs(roll - rhs.roll) < 0.0001) &&
               (std::abs(pitch - rhs.pitch) < 0.0001) &&
               (std::abs(yaw - rhs.yaw) < 0.0001);
    }

    inline bool operator!= (const pose3d_t& rhs) const
    {
        return (std::abs(x - rhs.x) >= 0.0001) ||
               (std::abs(y - rhs.y) >= 0.0001) ||
               (std::abs(z - rhs.z) >= 0.0001) ||
               (std::abs(roll - rhs.roll) >= 0.0001) ||
               (std::abs(pitch - rhs.pitch) >= 0.0001) ||
               (std::abs(yaw - rhs.yaw) >= 0.0001);
    }

    inline pose3d_t& operator= (const pose3d_t& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        roll = rhs.roll;
        pitch = rhs.pitch;
        yaw = rhs.yaw;
        return *this;
    }

};

}

#endif
