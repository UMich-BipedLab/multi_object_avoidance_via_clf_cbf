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
%% * File:        getControlParameters.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: get the pre-defined parameters
%%*******************************************************************************/
function opts = getControlParameters(opts)
    % general parameters
    opts.gamma = 1; % weight of orientation, used to compute distance
    opts.alpha = 10; % weight between omega and vy
    opts.beta = 1.2; % move equilibria outside of FoV (only <3,1>)

    % control parameters
    % k_r1, k_r2: how far cassie walks diagonally 
    opts.forward_params.k_r1 = 1; % height of the surface
    opts.forward_params.k_r2 = 5; % width of the inner cone
    opts.forward_params.k_delta1 = 0.1; %  height of the surface
    opts.forward_params.k_delta2 = 10; % speed of saturation 10


    % Jessy's
%     opts.forward_params.k_r1 = 1; % height of the surface
%     opts.forward_params.k_r2 = 1; % width of the inner cone
%     opts.forward_params.k_delta1 = 0.1; %  height of the surface
%     opts.forward_params.k_delta2 = 1; % curvature of the surface
end