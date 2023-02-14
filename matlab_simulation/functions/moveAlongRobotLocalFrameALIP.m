%%/* Copyright (C) 2013-2025, The Regents of The University of Michigan.
%% * All rights reserved.
%% * This software was developed in the Biped Lab (https://www.biped.solutions/)
%% * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
%% * be available under alternative licensing terms; contact the address above.
%% * Redistribution and use in source and binary forms, with or without
%% * modification, are permitted provided that the following conditions are met:
%% * 1. Redistributions of source code must retain the above copyright notice, this
%% *    list of conditions and the following disclaimer.
%% * 2. Redistributions in binary form must reproduce the above copyright notice,
%% *    this list of conditions and the following disclaimer in the documentation
%% *    and/or other materials provided with the distribution.
%% * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
%% * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
%% * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%% * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
%% * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
%% * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%% * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%% * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%% * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%% * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%% * The views and conclusions contained in the software and documentation are those
%% * of the authors and should not be interpreted as representing official policies,
%% * either expressed or implied, of the Regents of The University of Michigan.
%% */
%%/*******************************************************************************
%% * File:        moveAlongRobotLocalFrameALIP.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: describe the robot's walking dynamic model -- ALIP model
%%*******************************************************************************/
function new_pose = moveAlongRobotLocalFrameALIP(...
    opts, robot_pose, FCV_old, FCV)

    step_inerval = opts.time_interval;

    % transform vx, vy to world frame
    robot_theta = robot_pose(3);
    cos_theta = cos(robot_theta);
    sin_theta = sin(robot_theta);
    
    % initial velocity at world frame
    vx_w0 = FCV_old.vx_star * cos_theta - FCV_old.vy_star * sin_theta;
    vy_w0 = FCV_old.vx_star * sin_theta + FCV_old.vy_star * cos_theta;
    
    % target velocity at world frame
    target_vx_w = FCV.vx_star * cos_theta - FCV.vy_star * sin_theta;
    target_vy_w = FCV.vx_star * sin_theta + FCV.vy_star *cos_theta;
    
    g = 9.8;  %* cosd(40);
    H = 0.8;
    omega_alip = sqrt(g/H); % deg/s
    
    
    px = computeFootPosition(robot_pose(1), vx_w0, target_vx_w , omega_alip, step_inerval);
    x = computeCoMPosition(robot_pose(1), vx_w0, px, omega_alip, step_inerval);
    
    py = computeFootPosition(robot_pose(2), vy_w0, target_vy_w , omega_alip, step_inerval);
    y = computeCoMPosition(robot_pose(2), vy_w0, py, omega_alip, step_inerval);
    
    
    delta_theta_r = FCV.omega_star * opts.time_interval;
    theta_w = robot_pose(3) - delta_theta_r;
    new_pose = [x, y, theta_w];

end