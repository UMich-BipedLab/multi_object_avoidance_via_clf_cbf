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
%% * File:        computeDistance.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: compute distance with equation in paper
%%*******************************************************************************/
function d = computeDistance(opts, r, delta)
    if (opts.CLF_model == 1) % eq 6
        d = sqrt(r^2 + opts.gamma^2 * delta^2);
    elseif opts.CLF_model == 2 % eq 13
        d = (r^2 + opts.gamma^2 * sin(delta)^2) / 2;
    elseif opts.CLF_model == 3 % eq 16
        d = (r^2 + opts.gamma^2 * sin(opts.beta * delta)^2) / 2;
    else
        error("no such opts.CLF_model for now: %i", ...
            opts.CLF_model);
    end
    
    if ~isfield(opts, '.obs')
        opts.obs.obstacle = 0;
    end
    
    
    if opts.obs.obstacle
        x = opts.data.cur_pose(1);
        y = opts.data.cur_pose(2);
        dist_to_centroid = sqrt((x - opts.obs.x)^2 + (y - opts.obs.y)^2);
        if (dist_to_centroid - opts.obs.radius) > opts.obs.buffer
            obs_cost = 1 / (dist_to_centroid - opts.obs.radius);
        else
            obs_cost = opts.obs.obs_cost;
        end
        d = d + obs_cost;
    end
end