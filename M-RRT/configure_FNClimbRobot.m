%
% Default configuration for 5-DOF climbing robot
%
% by Leon Guanfeng Liu on 11/07/2020 built on top of the work by Olzhas Adiyatov
% 6/10/2013

conf = struct;

conf.delta_goal_point = 0.15;  %0.5;                                          % Radius of goal point
conf.delta_goal_angle = 0.85; 
conf.num_links = 6;                                                  % Number of links in the manipulator model
conf.max_ang = pi * ones(conf.num_links,1);                % Maximum joint angle for each joint
conf.min_ang = -pi * ones(conf.num_links,1);                % Minimum joint angle for each joint
conf.delta_ang_max = 40 * pi/ 180; % 40 * pi / 180 % 20 * pi / 180; %10 * pi / 180;                                 % Maximum joint angle change at each iteration
conf.delta_ang_neighbor = 40 * pi / 180;   %30 * pi / 180;                            % Maximum joint angle change at each neighbor search
conf.disp_interval = 100;                                           % Interval for progress monitoring
conf.init_conf = zeros(conf.num_links,1);                  % Initial configuration of the manipulator model
conf.step_div = 4;                                                  % Number of transition states between nodes, used for collision detection

conf.len_link    = 1*ones(conf.num_links, 1);                       % Array storing the length of the links
conf.half_total_length = 0.5 * sum(conf.len_link);
%conf.rev_len_link = conf.len_link(end:1);                          % reverse link length vector (because sometimes robot might seated at link n)
conf.height_link = 0.2 *ones(conf.num_links, 1);                      % Array storing the height of the links
conf.width_link  = 0.8 *ones(conf.num_links, 1);                       % Array storing the width of the links

% goal configuration, same as init_conf, but sitting on different holds
% so basically we want to the robot to moving from starting hold to goal
% hold with the same orientation (Note: this is not 
conf.goal_conf = pi/2 * ones(conf.num_links, 1); %conf.init_conf;

%% there are also configurations about which terminals of the arm grab the hold
conf.terminal = 0;  % means that link 0 grabs the initial hold
%% conf.goal_holder = 6 or 1 is undetermined, because it can use robot base or tip to grab the hold
%% but it will be determined during the planning before reaching the goal hold

conf.hold_orientation = pi/2;  % for vertical climbing, no matter holder = link 1
                               % or link 6, the orientation of the holder
                               % link is pi/2
                               
%%  conf.hold_orientation = pi/2 is also necessary for transtion from 2 hold mode to 1 hold mode
% by breaking contact at 1 hold, necessary for safe gaiting; in the future
% might also consider  conf.num_hold_links  = 2, meaning there are 2 links
% in conf.hold_orientation 

%% when a vertice from one obstacle has a distance from an edge from another obstacle
%% is less than this parameter, consider to form a possible narrow passage.
conf.narrow_dist = 1.5;  

%% used in knn search algorithm: number of neighbors for each hold
conf.neighbors = 3;

%% check the dot product of two unit vector, see if it is greater than half_planer_epsilon
% if it is, then their angles are < 90 degree, else, > 90 degree
conf.half_plane_epsilon = 0.01;

%% the gap between robot tip and desired hold, a limit for saying that the loop has been closed
conf.dvEpsilon = 0.1;

%% maximal iteration steps for closing the loop
conf.maxLoopCloseSteps = 200;

%% accordion motion coefficient for closing the loop
conf.accordion_coef = 0.5;

%% configures for plotting
conf.hold_radius = 0.2;

%% animation figure id (for avoiding conflict with existing plot figure ids, we need to configure a
% new id for animation purpose
conf.animation_fig_id = 500;
conf.clearance_fig_id = 600;

%% the following is for easier animation
conf.max_linkwidth = max(conf.width_link);

%% narrow link dist, Note: better to set as conf.narrow_dist - 0.5 * conf.max_linkwidth. But in the experiment
% we found that the current setting also gives good result
conf.narrow_link_dist = conf.narrow_dist - conf.max_linkwidth; 

%% joint linear interpolation step
conf.step_delta = 0.1; 

%% samples progressions in a plot (i.e. plotting one sample every n actual samples
conf.plot_samples_progression = 5;
conf.num_plots = 6;
conf.plot_sample_start_fig_id = 500;
conf.num_samples_in_one_plot = 15;
conf.epsilon = 0.0001;

%% animation using smooth traj or using raw traj
conf.animate_raw = true;

%% in searching nearest nodes on tree from a given sample, compare how many nearest nodes
conf.nearest_nodes_comp_numbers = 5;

%% probability for sampling topological components
conf.sample_topo = 0.5;  % 50%, purely randomly samling when conf.sample_topo= 1.0, purely topo sampling when conf.sample_topo = 0.0;

%% search for non-colliding conf for closed chain
conf.init_search_tries = 50;

%% whether to animate robot motion for every motion step
conf.animate_every_step = true;

%% whether to skip closed chain reconfig phse
conf.skip_closechain = true;

%% stop sampling if number of nodes on tree greater than a given number
conf.stop_number_samples_on_tree = 4000;

%% maximal iterations allowed
conf.total_iter = 2e6;

%% animation file name
conf.animation_filename = 'regular_rrt_test';