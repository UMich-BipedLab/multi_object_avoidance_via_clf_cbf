%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Jason Choi (jason.choi@berkeley.edu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%% * File:        ctrlCbfClfQp.m
%% *
%% * Author:      Minzhe Li (minzlee[at]umich.edu)
%% * Created:     
%% *
%% * Description: reproduce the clf-cbf-qp function
%%*******************************************************************************/
function [u, slack, B, V, feas, comp_time] = ctrlCbfClfQp(obj, x, robot_current_pose, u_ref, with_slack, verbose)
    %% Implementation of vanilla CBF-CLF-QP
    % Inputs:   x: state
    %           u_ref: reference control input
    %           with_slack: flag for relaxing the clf constraint(1: relax, 0: hard-constraint)
    %           verbose: flag for logging (1: print log, 0: run silently)
    % Outputs:  u: control input as a solution of the CBF-CLF-QP
    %           slack: slack variable for relaxation. (empty list when with_slack=0)
    %           B: Value of the CBF at current state.
    %           V: Value of the CLF at current state.
    %           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
    %           when qp is infeasible, u is determined from quadprog.)
    %           comp_time: computation time to run the solver.
    if isempty(obj.clf)
        error('CLF is not defined so ctrlCbfClfQp cannot be used. Create a class function [defineClf] and set up clf with symbolic expression.');
    end
    if isempty(obj.cbf)
        error('CBF is not defined so ctrlCbfClfQp cannot be used. Create a class function [defineCbf] and set up cbf with symbolic expression.');
    end
        
    if nargin < 4
        u_ref = zeros(obj.udim, 1);
    end
    if nargin < 5
        with_slack = 1;
    end

    if nargin < 6
        % Run QP without log in default condition.
        verbose = 0;
    end

    if size(u_ref, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end                
    
    obj.params.clf.rate = 0.02; 
    obj.params.cbf.rate = 0.4; 
    obj.params.weight_slack = 10;

    tstart = tic;
    V = obj.clf(x);
    LfV = obj.lf_clf(x);
    LgV = obj.lg_clf(x);
    
    B = obj.cbf(x);
    LfB = obj.lf_cbf(x);
    LgB = obj.lg_cbf(x);
    
    with_slack = 1;
    %% Constraints: A[u; slack] <= b
    if with_slack
        % CLF and CBF constraints.
        A = [LgV, -1;
        -LgB, 0];
        b = [-LfV - obj.params.clf.rate * V;
            LfB + obj.params.cbf.rate * B];                
        % Add input constraints if u_max or u_min exists.
        if isfield(obj.params, 'u_max')
            A = [A; eye(obj.udim), zeros(obj.udim, 1);];
            if size(obj.params.u_max, 1) == 1
                b = [b; obj.params.u_max * ones(obj.udim, 1)];
            elseif size(obj.params.u_max, 1) == obj.udim
                b = [b; obj.params.u_max];
            else
                error("params.u_max should be either a scalar value or an (udim, 1) array.")
            end
        end
        if isfield(obj.params, 'u_min')
            A = [A; -eye(obj.udim), zeros(obj.udim, 1);];
            if size(obj.params.u_min, 1) == 1
                b = [b; -obj.params.u_min * ones(obj.udim, 1)];
            elseif size(obj.params.u_min, 1) == obj.udim
                b = [b; -obj.params.u_min];
            else
                error("params.u_min should be either a scalar value or an (udim, 1) array")
            end
        end        
    else
        % CLF and CBF constraints.
        A = [LgV; -LgB];
        b = [-LfV - obj.params.clf.rate * V;
            LfB + obj.params.cbf.rate * B];                
        % Add input constraints if u_max or u_min exists.
        if isfield(obj.params, 'u_max')
            A = [A; eye(obj.udim)];
            if size(obj.params.u_max, 1) == 1
                b = [b; obj.params.u_max * ones(obj.udim, 1)];
            elseif size(obj.params.u_max, 1) == obj.udim
                b = [b; obj.params.u_max];
            else
                error("params.u_max should be either a scalar value or an (udim, 1) array.")
            end
        end
        if isfield(obj.params, 'u_min')
            A = [A; -eye(obj.udim)];
            if size(obj.params.u_min, 1) == 1
                b = [b; -obj.params.u_min * ones(obj.udim, 1)];
            elseif size(obj.params.u_min, 1) == obj.udim
                b = [b; -obj.params.u_min];
            else
                error("params.u_min should be either a scalar value or an (udim, 1) array")
            end
        end
    end

    obj.params.weight = 0;

    %% Cost
    if isfield(obj.params.weight, 'input')
        if size(obj.params.weight.input, 1) == 1 
            weight_input = obj.params.weight.input * eye(obj.udim);
        elseif all(size(obj.params.weight.input) == obj.udim)
            weight_input = obj.params.weight.input;
        else
            error("params.weight.input should be either a scalar value or an (udim, udim) array.")
        end
    else
        weight_input = eye(obj.udim);
    end
    
    if verbose
        options =  optimset('Display','notify');
    else
        options =  optimset('Display','off');
    end
    if with_slack         
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        weight_input(1,1) = 0.2;                       %weight for x-axis changes
        weight_input(3,3) = 0.2 * obj.params.alpha;    %weight for rotation changes
        H = [weight_input, zeros(obj.udim, 1);
            zeros(1, obj.udim), obj.params.weight_slack];
        f_ = [-weight_input * u_ref; 0];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
%         [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [-10;-10;-30], [10,10,30], [], options);
        if exitflag == -2
            feas = 0;
            disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
        else
            feas = 1;
        end
        u = u_slack(1:obj.udim);
        slack = u_slack(end);
    else
        % cost = 0.5 u' H u + f u
        alpha =  obj.params.alpha;
        weight_input = eye(obj.udim);
        H = weight_input;
        f_ = -1 * eye(obj.udim) * u_ref;
        [u, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [-10;-10;-30], [10,10,30], [], options);
        if exitflag == -2
            feas = 0;
            disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
        else
            feas = 1;
        end
        slack = [];
    end
    comp_time = toc(tstart);
end