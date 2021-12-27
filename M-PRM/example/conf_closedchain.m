%
% Default configuration for PRMs for 
%
% by Leon Guanfeng Liu on 11/07/2020 built on top of the work by Olzhas Adiyatov
% 6/10/2013

conf = struct;
% each point obstacle will be inflated into a polygon
conf.numVertex = 4;
conf.dilate_epsilon = 0.12;  % original 0.03, 8esp = 0.24

% link length vector
conf.linkLengthVec =[1.2000, 2.0000, 0.5512, 1.9457, 1.2131, 2.9482, 4.5684, 0.3000, 0.3000, 5, 2.5130,  8.5815];
% link width vector
conf.thickNess = 0.02;
conf.linkWidthVec = ones(1, 12) * conf.thickNess;

% number of point obstacles
conf.numObst = 4;

conf.ptObst(1).coord = [2.5,1.6]';
conf.ptObst(2).coord = [2.5,2.8]';
conf.ptObst(3).coord = [4.5 1.0]';
conf.ptObst(4).coord = [4.5,1.9]';

% start and goal configurations
conf.startc = [1.0631, 0.3865, 1.4542, -1.2802, -2.5472, 0.5913, -1.2102, 0.5527, 2.4609, 0.3069, 2.7463, 3.1416];
conf.endc = [2.2410, 0.8287, 1.0704, -0.2046, -1.6302, 0.2022, -2.2898, 1.1214, 1.6813, -0.2606, 1.1309, 3.1416];


conf.nobst_samples = 10; % each obstacle, sample how many contact or near contact (with C-obst) samples
conf.enable_bound = true; % sample boundary samples, i.e. link 1-link 2 align in the same direction
conf.neighbors= 10;  %% how many neighbors try to link one vertex to
conf.numCloses =50;  %% for building graph
conf.nonbsamples=100; %% how many regular samples are generated for each cycle in the main loop
conf.toposamples=400;  %% how many topo samples are generated for each cycle in the main loop
conf.max_iters = 1e4;

conf.use_fvc = true;  % if we are going to use free velocity cone algorithm
conf.narrow_dist = 1.11; %0.81;
conf.narrow_link_dist = conf.narrow_dist - conf.thickNess; 

conf.half_plane_epsilon = 0.01;
conf.step_delta = 2 * pi / 180;  %% 2 degree
conf.epsilon = 0.01;  % for determining the elbow-up or elbow-down torus in C-space parameterization
conf.rrv_narrow_numSamples = 5;