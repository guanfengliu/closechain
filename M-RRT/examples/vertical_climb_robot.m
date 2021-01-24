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
variant = 'FNClimbRobot';
%% planning vertical climbing function
result = climbPlanning(map, max_iter, is_benchmark, rand_seed, variant);
