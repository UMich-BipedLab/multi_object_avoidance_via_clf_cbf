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
%% * File:        computeFeedbackControlVariables.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: calculate the control variables based on the equation in paper
%%*******************************************************************************/
function out = computeFeedbackControlVariables(opts, r, delta, ...
                                                 v_r, v_delta)
    alpha = opts.alpha;
    A = constructAMatrix(r, delta);
    omega_star = computeOptimalOmega(alpha,delta,r,v_delta,v_r);
    vx_star = computeOptimalVx(alpha,delta,r,v_delta,v_r);
    vy_star = computeOptimalVy(alpha,delta,r,v_delta,v_r);
    
    
    out.A = A;
    out.omega_star = omega_star;
    out.vx_star = vx_star;
    out.vy_star = vy_star;   
end