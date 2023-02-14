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
%% * File:        single_obstacle_simulation.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     2021 Aug
%% *
%% * Description: simulation of one single obstacle performance
%%*******************************************************************************/

%% System setups
clear, clc
addpath(pwd,'lib','dynsys','functions');
addpath(genpath('matlab_utils'));
clear_fig = 0;

fig = createFigureOptions(1, 1, "simulate_clf_cbf_test", clear_fig, 1, 1);
fig.clear_fig = clear_fig;
if fig.clear_fig
    plot_count = 1;
    save('plot_count.mat', 'plot_count')
else
    load('plot_count.mat')
    plot_count = plot_count + 1;
    save('plot_count.mat', 'plot_count')
end
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);

h_traj = [];
h_x = [];
h_y = [];
h_theta = [];
h_vx = [];
h_vy = [];
h_omega = [];
h_vr = [];
h_vd = [];
h_d = [];
h_r = [];
h_CBFh = [];
h_CLFV = [];


%% parameters setting
test_case = 1;

% initialization
target_position = [0, 0];   % in w frame

if test_case == 1       % same start pose with different obstacle's location
    ini_poses(1).robot_init_pose = [-15, -15, deg2rad(-15)];
    alpha = 5;
    opts.objnum = 6;
    
    opts.singleObjFlag = 1;
    % add the objectives center
    opts.objradius(1) = 0;	% for the case without obstacle
    opts.objcenter(1,:) = [-2, -5];
    opts.objradius(2) = 1;
    opts.objcenter(2,:) = [-8,-14];
    opts.objradius(3) = 1;
    opts.objcenter(3,:) = [-5,-10];
    opts.objradius(4) = 1;
    opts.objcenter(4,:) = [-3,-7];
    opts.objradius(5) = 1;
    opts.objcenter(5,:) = [-10,-12];
    opts.objradius(6) = 1;
    opts.objcenter(6,:) = [-12,-15];
    
elseif test_case == 2       % same start position, different orientation, same obstacle place
    ini_poses(1).robot_init_pose = [-10, -10, deg2rad(75)];
    ini_poses(2).robot_init_pose = [-10, -10, deg2rad(60)];
    ini_poses(3).robot_init_pose = [-10, -10, deg2rad(44)];
    ini_poses(4).robot_init_pose = [-10, -10, deg2rad(30)];
    ini_poses(5).robot_init_pose = [-10, -10, deg2rad(15)];
    alpha = 5;
    opts.objnum = 1;
    opts.singleObjFlag = 1;
    opts.objradius(1) = 1;  % for the case without obstacle
    opts.objcenter(1,:) = [-4, -4];
    
elseif test_case == 3       % multi-obstacle test case
    ini_poses(1).robot_init_pose = [-15, -15, deg2rad(-15)];
    alpha = 5;
    opts.objnum = 3;
    opts.singleObjFlag = 0;
    opts.objradius(1) = 2;
    opts.objcenter(1,:) = [-2, -5];
    opts.objradius(2) = 1;
    opts.objcenter(2,:) = [-8,-14];
    opts.objradius(3) = 1;
    opts.objcenter(3,:) = [-5,-10];
    
elseif test_case == 4    % single obstacle, single run
    ini_poses(1).robot_init_pose = [-15, -15, deg2rad(-15)];
    alpha = 5;
    opts.objnum = 1;
    
    opts.singleObjFlag = 1;
    opts.objradius(1) = 1;
    opts.objcenter(1,:) = [-2, -5];
end

% time
opts.time_interval = 0.05;  % in seconds
opts.time_duration = 60;    % in seconds

enlarge = 1;
vx_upperbound = 0.7;
vy_upperbound = 0.2;

colors = getColors(20);     %number of colors available to use
usingCBFFlag = 1;	%use clf-cbf-qp

% user parameters
opts.target_position = target_position;
opts.epsilon = 1e-8;
opts.reach_goal_threshold = 0.05;
opts.CLF_model = 3;
opts.forward_model = 1;
opts.obs.buffer = 0.1;
opts.rotate_in_place_omega = deg2rad(5);    % rad/s
opts = getControlParameters(opts);
opts.plot_control_variables = 0;
opts.u_ref = [0; 0; 0];
opts.alpha = alpha;
opts.objcycleNow = 1;


%% run the test scenario
if opts.singleObjFlag
    objcycleLength = opts.objnum;
else
    objcycleLength = 1;
end

for p = 1:objcycleLength
    opts.objcycleNow = p;
    cassie = CassieRob(opts);  %set up the robot and qp system
    controller = @cassie.ctrlCbfClfQp;
    
    for k = 1 : length(ini_poses)
        failFlag = 0;
        color = colors{(p-1)*length(ini_poses)+k};    % change number each time for different test
        clear results;
        disp("==========================")
        fprintf("Working on %i/%i\n", k, length(ini_poses))
        disp("==========================")
        robot_init_pose = ini_poses(k).robot_init_pose;
        
        
        time = 0 : opts.time_interval : opts.time_duration;
        num_points = length(time)
        
        
        disp("Initial r and delta")
        [r, delta] = computeEgoPolorCoordinate(...
            opts, robot_init_pose, target_position);
        fprintf("r: %.2f\n", r)
        fprintf("delta: %.2f\n", rad2deg(delta))
        equilibria = pi/(2 * opts.beta);
        fprintf("equilibria: " + char(177) +"%.2f\n", rad2deg(equilibria))
        robot_rotated_pose = robot_init_pose;
        distance_to_manifold = rad2deg(computeDistanceToManifold(opts, delta))
        
        
        robot_rotated_pose = rotateInPlace(opts, robot_init_pose, target_position);
        [r, delta] = computeEgoPolorCoordinate(...
            opts, robot_rotated_pose, target_position);
        disp("Rotated r and delta")
        fprintf("r: %.2f\n", r)
        fprintf("delta: %.2f\n", rad2deg(delta))
        fprintf("equilibria: " + char(177) +"%.2f\n", rad2deg(pi/(2 * opts.beta)))
        distance_to_manifold = rad2deg(computeDistanceToManifold(opts, delta))
        
        
        results(num_points) = struct('t', [], 'x_r', [], 'y_r', [], ...
            'r', [], 'delta', [], ...
            'distance', [], 'v_r', [], 'v_delta', [], ...
            'A', [], 'v_x', [], 'v_y', [], 'omega', [], 'theta', [], ...
            'pose',[], 'pose_with_omega', [], 'new_pose', [], 'CBFh', []);
        
        time_index = 0;
        robot_current_pose = robot_rotated_pose;
        FCV_old = computeFeedbackControlVariables(opts, r, delta, 0, 0);
        
        
        for i = 1:num_points
            % compute ego polor coordinate in local frame
            [r, delta] = computeEgoPolorCoordinate(...
                opts, robot_current_pose, target_position);
            
            % compute distance from pose to point
            opts.obs.obstacle = 0; % 0
            distance = computeDistance(opts, r, delta);
            
            % compute the distance to the objective
            [r_obj, delta_obj] = computeEgoPolorCoordinate(...
                opts, robot_current_pose, opts.objcenter(p,:));
            %     opts.obs.obstacle = 1;
            distanceToObj = computeDistance(opts, r_obj, r_obj);
            
            
            %% clf/u_ref calculation
            % compute forward control variables
            [v_r, v_delta] = computeForwardControlVariables(opts, r, delta);
            
            
            % compute feedback control variables (v_x and v_y)
            FCV = computeFeedbackControlVariables(opts, r, delta, v_r, v_delta);
            theta = atan2(FCV.vy_star, FCV.vx_star); % angle between v_x and v_y
            
            %% considering the obstacles in the environment
            if usingCBFFlag == 1
                opts.u_ref = [FCV.vx_star; FCV.vy_star; FCV.omega_star];
                opts.robot_current_pose = robot_current_pose;
                currTheta = robot_current_pose(3);
                [u, slack, h, V] = controller([r; delta; currTheta], robot_current_pose, opts.u_ref);
                
                % for regular value check
                %         u
                %         h
                %         robot_current_pose
                FCV.vx_star = u(1);
                FCV.vy_star = u(2);
                FCV.omega_star = u(3);
                results(i).CBFh = h;
                results(i).CLFV = V;
            end
            
            if enlarge == 1
                vx_scale = abs(vx_upperbound / FCV.vx_star);
                vy_scale = abs(vy_upperbound / FCV.vy_star);
                scaling = min(vx_scale, vy_scale);
                FCV.vx_star = scaling * FCV.vx_star;
                FCV.vy_star = scaling * FCV.vy_star;
                FCV.omega_star = scaling * FCV.omega_star;
            end
            
            %% the rest propagation
            % using ALIP model
            new_pose = moveAlongRobotLocalFrameALIP(...
                opts, robot_current_pose, FCV_old, FCV); % in w frame
            FCV_old = FCV;
            
            % pack to result structure
            results(i) = packToResults(results(i), ...
                new_pose(1), new_pose(2), ...
                r, delta, distance, v_r, v_delta, ...
                FCV.A, FCV.vx_star, FCV.vy_star, FCV.omega_star, new_pose(3));
            results(i).t = time(i);
            results(i).theta = new_pose(3);
            results(i).new_pose = new_pose;
            
            robot_current_pose = new_pose; % in w frame
            opts.robot_current_pose  = new_pose;
            if (computeEuclideanPoseDistance(new_pose, target_position) < ...
                    opts.reach_goal_threshold)
                time_index = i;
                break
            end
            % print out i as the progress checking
            i
        end
        
        %% extra break guarantee
        if i == num_points
            time_index = i;
        end
        
        
        %% plottings
        for i = 1:20:num_points
            if (i > time_index)
                break;
            end
            h0 = plotRobotPose2D(axes_handles(1), results(i).new_pose, color, 0.5);
        end
%         h1 = plotRobotPose2D(axes_handles(1), robot_init_pose, 'r', 1);
%         h2 = plotRobotPose2D(axes_handles(1), robot_rotated_pose, 'm', 1);
        hold on
        h3 = scatter(axes_handles(1), target_position(1), target_position(2), 100, 'k', 'fill');
        [eq_num, sol_num] = getEquationNumbers(opts);
        showCurrentPlot(axes_handles(1), ...
            "simulation trajectory", ...
            [0, 90], 1)
        xlabel(axes_handles(1), "x [m]", 'fontsize',18)
        ylabel(axes_handles(1), "y [m]", 'fontsize',18)
        drawnow
        
        
        %% draw the obstacle
        if usingCBFFlag ~= 0
            if opts.singleObjFlag
                draw_circle(axes_handles(1), opts.objcenter(p,:), opts.objradius(p), color);
            else
                for cir = 1:opts.objnum
                    draw_circle(axes_handles(1), opts.objcenter(cir,:), opts.objradius(cir), color);
                end
            end
        end
        
        % t=title("Robot Trajectory in Multiple Obstacles Environment");
        t.FontSize = 18;
        
        
    end
end

% draw circle obstacle
function h = draw_circle(handles,center,r, color)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(handles, xunit, yunit, 'Color', color, 'LineWidth', 2);
hold off
end