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
%% * File:        computeEgoPolorCoordinate.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: compute variables in ego polor coordinates
%%*******************************************************************************/
function [r, delta] = computeEgoPolorCoordinate(otps, ...
    robot_pose, target_position)
%     dx = (x_t - x_r) + otps.epsilon;
%     dy = (y_t - y_r) + otps.epsilon;
    dx = (target_position(1) - robot_pose(1));
    dy = (target_position(2) - robot_pose(2));
    r = sqrt(dx^2 + dy^2);
    
    
    if (abs(dx) < otps.epsilon)
        if (dy > 0)
            phi = pi/2;
        else
            phi = -pi/2;
        end
    else
        phi = (atan2(dy + otps.epsilon, dx));
    end
    delta = wrapToPi(phi - robot_pose(3));
%     delta = abs(delta);

%     delta = atan2(abs(dy) + otps.epsilon, abs(dx) + otps.epsilon);

end