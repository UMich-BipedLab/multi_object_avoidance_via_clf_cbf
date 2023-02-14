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
%% * File:        defineCbf.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: define the control barrier function for the system
%%*******************************************************************************/
function cbf = defineCbf(~, opts, symbolic_state)
    derivDistance = 0;
    distance1 = 1;
    k = 1/3;
    if opts.singleObjFlag
        cyclestart = opts.objcycleNow;
        cycleend = opts.objcycleNow;
    else
        cyclestart = 1;
        cycleend = opts.objnum;
    end
        
    for i = cyclestart : cycleend
        obj_d = opts.objradius(i);
        objcenter = opts.objcenter(i,:);

        r1 = symbolic_state(1);
        delta1 = symbolic_state(2);
        theta1 = symbolic_state(3);

        xrobot_current_pose_by_rdelta = opts.target_position(1) -  r1 *  cos(delta1 + theta1);
        yrobot_current_pose_by_rdelta = opts.target_position(2) -  r1 *  sin(delta1 + theta1);

        dx = (objcenter(1) - xrobot_current_pose_by_rdelta);
        dy = (objcenter(2) - yrobot_current_pose_by_rdelta);
        dis = (dx^2 + dy^2 - obj_d^2);
        cost = dis^k;       
        distance1 = distance1 * cost;    
    end
    cbf = distance1 + (derivDistance);
end