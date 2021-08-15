% algorithm for 6 dof redundant robot vertical climbing, in which
% there are algorithms for searching reachable holds and create
% a sequence of discrete hold state sequences, and an RRT algorithm
% between 2 consecutive hold. The RRT uses our core ranomly sampling
% algorithm that samples topological components (i.e. uses global info
% of C-free)
% by Leon Guanfeng Liu
% 25-10-2020

%% in the following robot start point is the actual point at which the robot
%% is based at, recall
%% robot link length vector and chain link indice vector (link 1- link n or
%% reversed order link n - link 1) is set up in configure_climb_robot.m
%% 
map = struct('name', 'climb_map5.mat', 'start_hold_index', 1, 'goal_hold_index', 9);
max_iter = 6e4; %20e3;
is_benchmark = false;
rand_seed = 40;
variant = 'FNClimbRobot_RRV';
% sample_alg is the type of the sampling
% sample_alg = 1;  
%% planning vertical climbing function
if strcmp(variant, 'FNClimbRobot')        %%% Regular RRT with/without combined topological samples + regular samples
  result = climbPlanning(map, max_iter, is_benchmark, rand_seed, variant);
elseif strcmp(variant, 'FNClimbRobot_CONNECT')  %%% RRT-connect with/without combined topological samples + regular samples
    result = climbPlanning_CONNECT(map, max_iter, is_benchmark, rand_seed, variant);
elseif strcmp(variant, 'FNClimbRobot_BICONNECT')      %%************* note: we haven't supported this option yet  *************
    result = climbPlanning_BICONNECT(map, max_iter, is_benchmark, rand_seed, variant);
elseif strcmp(variant, 'FNClimbRobot_RRV')   %%%%  RRV with/without combine topological samples/Jacobian based projection + regular samples          
    result = climbPlanning_RRV(map, max_iter, is_benchmark, rand_seed, variant);
end