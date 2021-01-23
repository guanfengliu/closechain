function problem = climbPlanning(map, max_iter, is_benchmark, rand_seed, variant)
% climbing algorithm is composed of a planning algorithm that searches a 
% a sequence of hold points; and then search searches path between two
% consecutive hold points.
% can we apply unified algorithm or we can divide into a discrete graph planning phase
% followed by an regular RRT search phase. In this version, we apply 2-phase algorithm 

%
% problem = climbPlanning(map, max_iter, is_benchmark, rand_seed, variant)
% function returns the object of the respective class with the result
%
% map           -- struct with appropriate fields (developer of 
%               the class provides more information on this topic)
% max_iter      -- number of iteration to solve the problem
% is_benchmark  -- if true saves snapshots of the tree in a special directory
%               boolean variable
% rand_seed     -- a random seed 
% variant       -- what class to choose, class used defines the problem space
% 
%
%
% Leon Guanfeng Liu on 10/28/2020 based upon the original RRT function developed 
% by Olzhas Adiyatov
% 05/13/2013

%%% Configuration block
if nargin < 5
    disp('ERROR: the number of input parameters should be 5')
    return;
else
    MAX_NODES   = max_iter;
    MAX_ITER    = max_iter;
    RAND_SEED   = rand_seed;
    MAP         = map;
end

addpath(genpath(pwd));

% loading settings
if exist(['configure_' variant '.m'], 'file')
    run([pwd '/configure_' variant '.m']);
    CONF = conf;
else
    disp('ERROR: There is no configuration file!')
    return
end

ALGORITHM = 'RRT';

problem = eval([variant '(RAND_SEED, MAX_NODES, MAP, CONF);']);

%%% Starting a timer
tic;

%%% we also need to record successful RRT paths between given pair of nodes
% RRT_path_map is a structure array to record all computed RRT paths beween
% pairs of holds
RRT_path_map = [];
num_RRT_paths = 0;


%%% find all path between start and goal hold points
holdPaths = problem.searchHoldSequence();
num_holdPaths = length(holdPaths);

%% start joint/cartesian and desired joint/cartesian configuration
[start_j, start_c] = problem.getStartConfig();
%% desired_c is undetermined at this stage, it is determined by
% the actual terminal (link 0 or link n+1) when reach goal_hold
[desired_j] = problem.getDesiredConfig();

%final_j_path=[];
%final_c_path=[];
for i=1:num_holdPaths
    singleHoldPath = holdPaths{i};
    pathLength = length(singleHoldPath);
    hold_l = problem.getStartHoldid();
    last_j = start_j;
    %% final joint and cartesian path (raw)
    final_j_path = [];
    final_c_path = [];
    %% vector of 0/1 indicate whether a raw sample is 
    final_narrow_topo = [];
    final_smooth_j_path = [];
    final_smooth_c_path = [];
    length_j_array = []; % length array for plotting, each component representing the number of samples in a given planning step
    length_smooth_j_array = []; % length array for smoothing samples
   
    %final_t_path = [];
    final_narrow_samples_j = [];
    final_narrow_samples_c = [];
    final_path_clearance_raw = [];
    % statistics for how many narrow samples from topo components, and how
    % many narrow samples from regular radom samples
%     numNarrowFromTopo = 0;
%     numNarrowFromRand = 0;
    
    %path_j3 =[]; path_c3 = []; path_t3=[]; % path_t3 is the list of terminals that robot uses
    terminal = 0;  % robot based at left initially
    for j=2:pathLength+1
        if j < pathLength+1
          curHold = singleHoldPath(j);
        else
          curHold = singleHoldPath(end);
        end
        path_j =[]; path_c = []; %path_t=[]; 
        narrow_topo = [];
        smooth_j =[]; smooth_c = [];
        narrow_samples_j=[]; narrow_samples_c=[]; % path_t3 is the list of terminals that robot uses
        path_clearance_raw =[];
        % check if path between the two holds already computed 
        [is_path_already_computed, RRT_path_id] = checkIfRRTPathExist(RRT_path_map, hold_l, curHold, terminal);
        if  ~is_path_already_computed   
            %% normal gait step (robot moves from hold_l or curHold
            if hold_l ~= curHold  
               %% update start hold, goal hold, and move robot from last_j to any config that grabs curHold 
               mode = "open";
               %% desired_j=-1 means we don't set desired_j, and let the problem itself determine the desired_j
               status = problem.updateTree(hold_l, curHold, terminal, mode, last_j, -1);
               if ~status
                   fprintf("path %d is not feasible, update tree fails with left_hold_id=%d, right_hold_id=%d, terminal=%d, mode=%s",...
                           j, hold_l, curHold, terminal, mode);
                   break;
               end
               %% after updating RRT tree configs, compute the RRT paths between them
               [path_j1, path_c1, narrow_topo1, smooth_j1, smooth_c1, narrow_samples_j1,...
                   narrow_samples_c1, path_clearance_raw1] = applyRRT(true, problem, 2 * MAX_ITER, is_benchmark, CONF);
               if length(path_j1)==0 || length(path_c1)==0
                  %% this path is not possible
                  fprintf("path %d is not feasible, no path between hold %d and hold %d \n", j, hold_l, curHold);
                  break;
               end
               %% need to close the loop before performing the closed-chain mode motion
%                [j1, c1] = problem.closeLoopByPseudoIK(path_j1(:,end));
%                if size(j1,2)>0 
%                    if ~problem.CheckCollision(j1(:,end))
%                       path_j3 = [path_j3, path_j1, j1];
%                       path_c3 = [path_c3, path_c1, c1];
%                       problem.Animation_path(path_c3, 'test_path_rrt_vert');
%                    else
%                       path_j3 = [path_j3, path_j1];
%                       path_c3 = [path_c3, path_c1];
%                       fprintf("path segment %d is not feasible, closeLoop by pseudo IK leads to a closed loop configuration with collision \n");
%                       %break;
%                       fprintf("so we skip configuration by pseudo IK \n"); 
%                    end
%                else
%                   path_j3 = [path_j3, path_j1];
%                   path_c3 = [path_c3, path_c1];
%                   fprintf("path segment %d is not feasible, closeLoop by pseudo IK has no solution \n");
%                   %break;
%                   fprintf("so we skip configuration by pseudo IK \n");
%                end
               %path_t3 = [path_t3, terminal * ones(1, size(path_j3,2))];
               %raw samples
               path_j = [path_j, path_j1];
               path_c = [path_c, path_c1];
               narrow_topo = [narrow_topo, narrow_topo1];
               numRawSamples = size(path_j,2);
               length_j_array = [length_j_array, numRawSamples];
               narrow_samples_j = [narrow_samples_j, narrow_samples_j1];
               narrow_samples_c = [narrow_samples_c, narrow_samples_c1];
               %smoothing samples
               smooth_j= [smooth_j, smooth_j1];
               smooth_c = [smooth_c, smooth_c1];
               numSmoothSamples = size(smooth_j,2);
               length_smooth_j_array = [length_smooth_j_array, numSmoothSamples];
               
               path_clearance_raw = [path_clearance_raw, path_clearance_raw1];
               %% update that start_hold=goal_hold=curHold, so in closed chain mode
               %% robot has to internal adjust shape so that it stays stable in curHold (defined by config parameter in problem object)
               if ~CONF.skip_closechain
                 mode = "closed";
                 status = problem.updateTree(hold_l, curHold, terminal, mode, path_j(:,end), -1);
                 if ~status
                    fprintf("path %d is not feasible, update tree fails with left_hold_id=%d, right_hold_id=%d, terminal=%d, mode=%s \n",...
                           j, hold_l, curHold, terminal, mode);
                    break;
                 end
                 [path_j2, path_c2, narrow_topo2, smooth_j2, smooth_c2, narrow_samples_j2,...
                    narrow_samples_c2, path_clearance_raw2] = applyRRT(false, problem, 2 * MAX_ITER, is_benchmark, CONF);
                 if length(path_j2)==0 || length(path_c2)==0
                    %% this path is not possible
                    fprintf("path %d is not feasible, no path to adjust internal shape of the closed chain between hold %d and hold %d\n", j, hold_l, curHold);
                    break;
                 end
                 path_j = [path_j, path_j2];
                 path_c = [path_c, path_c2];
                 narrow_topo = [narrow_topo, narrow_topo2];
                 length_j_array = [length_j_array, size(path_j,2)-numRawSamples];
                 narrow_samples_j = [narrow_samples_j, narrow_samples_j2];
                 narrow_samples_c = [narrow_samples_c, narrow_samples_c2];
                 %smoothing samples
                 smooth_j= [smooth_j, smooth_j2];
                 smooth_c = [smooth_c, smooth_c2];
                 length_smooth_j_array = [length_smooth_j_array, size(smooth_j,2) -numSmoothSamples];
                 path_clearance_raw = [path_clearance_raw, path_clearance_raw2];
               end
                 %path_t = terminal * ones(1, size(path_j,2));
            else
               %% the very last gait step
               % here "open", and from curHold to curHold (this means
               % the last motion at the goal hold for adjusting the
               % config to the desired config
               status = problem.updateTree(hold_l, curHold, terminal, "open", last_j, desired_j);
                if ~status
                   fprintf("path %d is not feasible, update tree fails with left_hold_id=%d, right_hold_id=%d, terminal=%d, mode=%s \n",...
                           j, hold_l, curHold, terminal, mode);
                   break;
               end
               [path_j, path_c, narrow_topo, smooth_j, smooth_c, narrow_samples_j,...
                   narrow_samples_c, path_clearance_raw] = applyRRT(false, problem, 2 * MAX_ITER, is_benchmark, CONF);
               if length(path_j)==0 || length(path_c)==0
                  %% this path is not possible
                  fprintf("last path segment is not feasible at goal hold %d\n", curHold);
                  return;
               end
               length_j_array = [length_j_array, size(path_j,2)];
               length_smooth_j_array = [length_smooth_j_array, size(smooth_j,2)];
               %path_t = terminal * ones(1, size(path_j,2));
            end
            %% record the above two segment paths combined into the RRT_path_map structure array
            num_RRT_paths = num_RRT_paths + 1;
            RRT_path_map(num_RRT_paths).hold_l = hold_l;
            RRT_path_map(num_RRT_paths).hold_r = curHold;
            RRT_path_map(num_RRT_paths).path_j = path_j;
            RRT_path_map(num_RRT_paths).path_c = path_c;
            RRT_path_map(num_RRT_paths).smooth_j = smooth_j;
            RRT_path_map(num_RRT_paths).smooth_c = smooth_c;
            RRT_path_map(num_RRT_paths).narrow_topo = narrow_topo;
            RRT_path_map(num_RRT_paths).narrow_samples_j = narrow_samples_j;
            RRT_path_map(num_RRT_paths).narrow_samples_c = narrow_samples_c;
            RRT_path_map(num_RRT_paths).path_clearance_raw = path_clearance_raw;
            %% in general robot moves from hold_l to hold_r might take
            %% different path given different starting terminal.
            %% i.e. if robot link 0 based at hold_l and robot moves to grab hold_r with link n+1
            %% is different from that robot link n+1 based at hold_l, and robot moves to grab hold-r with link 0
            RRT_path_map(num_RRT_paths).terminal = terminal;
            %% update start and end joint angle vector for this num_RRT_paths segment
            %RRT_path_map(num_RRT_paths).start_j = path_j3(:, 1);
            %RRT_path_map(num_RRT_paths).end_j = path_j3(:, end);
        else
            
            path_j = RRT_path_map(RRT_path_id).path_j;
            path_c = RRT_path_map(RRT_path_id).path_c;
            narrow_samples_j = RRT_path_map(RRT_path_id).narrow_samples_j;
            narrow_samples_c = RRT_path_map(RRT_path_id).narrow_samples_c;
            narrow_topo = RRT_path_map(RRT_path_id).narrow_topo;
            smooth_j = RRT_path_map(RRT_path_id).smooth_j;
            smooth_c = RRT_path_map(RRT_path_id).smooth_c;
            path_clearance_raw = RRT_path_map(RRT_path_id).path_clearance_raw;
            desired_j =  path_j(:,1);
            if norm(desired_j - last_j) > problem.getNeighborDiff()
               % need to plan a motion from last_j to first_j3
               status = problem.updateTree(hold_l, curHold, terminal, "open", last_j, desired_j);
               if ~status
                   fprintf(["path %d is not feasible, although a pre-computed path exist, but update tree",
                       "in computing prepath fails with left_hold_id=%d, right_hold_id=%d, terminal=%d, mode=%s \n"],...
                           j, hold_l, curHold, terminal, mode);
                   break;
               end
               [pre_path_j, pre_path_c, pre_narrow_topo, pre_smooth_j, pre_smooth_c, pre_narrow_samples_j,...
                   pre_narrow_samples_c, pre_path_clearance_raw] = applyRRT(false, problem, 2 * MAX_ITER, is_benchmark, CONF);
               if length(pre_path_j)==0 || length(pre_path_c)==0
                  %% this path is not possible
                  fprintf(["path %d the prepath before using the existing path",...
                          "between left_hold_id=%d, and right_hold_id=%d is not feasible\n"],...
                      j, hold_l, curHold);
                  return;
               end 
               path_j = [pre_path_j, path_j];
               path_c = [pre_path_c, path_c];
               narrow_topo = [pre_narrow_top, narrow_topo];
               narrow_samples_j = [pre_narrow_samples_j, narrow_samples_j];
               narrow_samples_c = [pre_narrow_samples_c, narrow_samples_c];
               smooth_j = [pre_smooth_j, smooth_j];
               smooth_c = [pre_smooth_c, smooth_c];
               path_clearance_raw = [pre_path_clearance_raw, path_clearance_raw]; 
            end
            length_j_array = [length_j_array, size(path_j,2)];
            length_smooth_j_array = [length_smooth_j_array, size(smooth_j,2)];
            %path_t3 = terminal * ones(1, size(path_j3,2));
        end
        % update last_j
        last_j = path_j(:, end);
        last_j = ConvertNormal(last_j(end:-1:1)+ pi);  % because we change the robot base, so the starting angle is different
        % update hold_l
        hold_l = curHold;
        final_j_path = [final_j_path, path_j];
        final_c_path = [final_c_path, path_c];
        %final_t_path = [final_t_path, path_t];
        final_narrow_samples_j = [final_narrow_samples_j, narrow_samples_j];
        final_narrow_samples_c = [final_narrow_samples_c, narrow_samples_c];
        
        final_narrow_topo = [final_narrow_topo, narrow_topo];
        final_smooth_j_path = [final_smooth_j_path, smooth_j];
        final_smooth_c_path = [final_smooth_c_path, smooth_c];
        
        final_path_clearance_raw = [final_path_clearance_raw, path_clearance_raw];
        terminal = ~terminal;
    end
    if j==pathLength+1
          %% annimation of the path
          length_j_array = cumsum(length_j_array);
          length_smooth_j_array = cumsum(length_smooth_j_array);
          %% print out important informations
          fprintf("Total number of samples in the path = %d \n", length_j_array(end));
          fprintf("Total number of samples in the smoothing path =%d \n", length_smooth_j_array(end));
          fprintf("Total number of narrow samples based upon topological component sampling in the path = %d \n", sum(final_narrow_topo));
          fprintf("Number of narrow samples on RRT trees based upon topological components = %d \n", size(final_narrow_samples_j,2));
          if CONF.animate_raw
              problem.Animation_path_j(final_j_path, final_c_path, final_narrow_topo, CONF.animation_filename);
              problem.make_print_j(final_j_path, final_c_path, final_narrow_topo, length_j_array);
          else
              problem.Animation_path_j(final_smooth_j_path, final_smooth_c_path, zeros(1, size(final_smooth_j_path,2)), CONF.animation_filename);
              problem.make_print_j(final_smooth_j_path, final_smooth_c_path, zeros(1, size(final_smooth_j_path,2)), length_smooth_j_array);
          end
          % plot path clearance
          problem.plot_path_clearance(final_narrow_topo, final_path_clearance_raw);
          %problem.Animation_path(final_c_path, 'test_path_rrt_vert');
          %problem.make_print(final_c_path, length_j_array);
          
          % save final_j_path, final_c_path, final_narrow_samples
          dlmwrite('j_path.txt', final_j_path);
          dlmwrite('c_path.txt', final_c_path);
          dlmwrite('narrow_topo.txt', final_narrow_topo);
          dlmwrite('smooth_j_path.txt', final_smooth_j_path);
          dlmwrite('smooth_c_path.txt', final_smooth_c_path);
          dlmwrite('narrow_samples_j.txt', final_narrow_samples_j);
          dlmwrite('narrow_samples_c.txt', final_narrow_samples_c);
          % draw figures for paper
          break;
    end
end
end

function [is_path_already_computed, RRT_path_id] = checkIfRRTPathExist(RRT_path_map_, hold_l_, hold_r_, terminal_)
   numRRTPaths = length(RRT_path_map_);
   for i=1:numRRTPaths,
       if RRT_path_map_(i).hold_l == hold_l_ &&...
          RRT_path_map_(i).hold_r == hold_r_ &&...
          RRT_path_map_(i).terminal == terminal_
          is_path_already_computed = true;
          RRT_path_id = i;
          return;
       end
   end
   is_path_already_computed = false;
   RRT_path_id = -1;
   return;
end

%%  main  RRT algorithm, baseEndPts means desired angle is determined by the alg itself until
% tip reaches desired hold
function [path_j, path_c, narrow_topo, smooth_j, smooth_c, narrow_samples_j, narrow_samples_c,...
    path_clearance_raw] = applyRRT(baseEndPts, problem, MAX_ITER, is_benchmark, CONF)
  path_j=[]; path_c=[]; narrow_topo=[]; smooth_j=[]; smooth_c=[]; narrow_samples_j=[]; narrow_samples_c=[];
  path_clearance_raw =[];
  numFromRand = 0;  % number of samples form purely random sampling
  numFromTopo = 0;  % number of samples from topological components
  numNarrowFromRand = 0;  % how many samples from purely random sampling are also narrow from minimal clearance
  numNarrowFromTopo = 0;  % how many samples from topological components are also narrow from minimal clearance
  numSamplesOnTree = 0;  % how many samples on the RRT tree
  numNarrowOnTree = 0;  % number of narrow samples in the generated tree, note that if there is a configuration on tree that is narrow
                        % then steering from this one 
  numNarrowOnTreeFromTopo = 0; %number of narrow samples on Tree that comes from topological component sampling
  narrow_samples_j = [];  % narrow samples coming from topological component sampling
  narrow_samples_c = [];  % cartesian of the above
  if(is_benchmark)
     benchmark_record_step = 250; % set step size of benchmark 250 by default
     benchmark_states = cell(MAX_ITER / benchmark_record_step, 1);
     timestamp = zeros(MAX_ITER / benchmark_record_step, 1);
     iterstamp = zeros(MAX_ITER / benchmark_record_step, 1);
  end
  %%% RRT starts here
  total_iter = 0;
  reach_goal = false;
  while ~reach_goal && total_iter < CONF.total_iter
  for ind = 1:MAX_ITER
    rnumber = rand(1);
    topo = false;
    if rnumber >= CONF.sample_topo
        % 10% sample narrow configurations
        [new_node, new_node_c] = problem.sampleTopo();
        topo = true;
        num_samples = size(new_node, 2);
        for i=1:num_samples,
          numFromTopo = numFromTopo + 1;
          [minClearance, isNarrow] = problem.calNarrowNess(new_node(:,i));
          if isNarrow
              numNarrowFromTopo = numNarrowFromTopo + 1;
          end
        end
    else
        % 90% sample regular configurations
        new_node = problem.sample();
        num_samples = size(new_node, 2);
        for i=1:num_samples
          numFromRand = numFromRand + 1;
          [minClearance, isNarrow] = problem.calNarrowNess(new_node(:,i));
          if isNarrow
              numNarrowFromRand = numNarrowFromRand + 1;
          end
        end
    end
    
    for i=1:num_samples,
       sample = new_node(:,i);
       nearest_node = problem.nearest(sample);
       sample = problem.steer(nearest_node, sample);
       % for close chain, the usual steering might be fails
       if size(sample,2) > 0
         if(~problem.obstacle_collision(sample, nearest_node))
            [minClearance, isNarrow] = problem.calNarrowNess(sample);
            if isNarrow
               numNarrowOnTree = numNarrowOnTree + 1;  % these are narrow points on trees
               if topo
                  narrow_samples_j = [narrow_samples_j, sample];
                  new_node_c = problem.generate_link_position(sample);
                  narrow_samples_c = [narrow_samples_c, new_node_c];
                  numNarrowOnTreeFromTopo = numNarrowOnTreeFromTopo + 1;
               end
            end 
            problem.insert_node(nearest_node, sample, isNarrow && topo);
            numSamplesOnTree = numSamplesOnTree + 1;
         end
       end
    end
    if is_benchmark && (mod(ind, benchmark_record_step) == 0)
        benchmark_states{ind/benchmark_record_step} = problem.copyobj();
        timestamp(ind/benchmark_record_step) = toc;
        iterstamp(ind/benchmark_record_step) = ind;
    end
    
    if(mod(ind, 100000) == 0)
        disp([num2str(ind) ' iterations ' num2str(problem.nodes_added-1) ' nodes in ' num2str(toc)]);
    end
  end
  [backtrace_path, path_iter, reach_goal] = problem.evaluate_path(baseEndPts);
  total_iter = total_iter + MAX_ITER;
  end
  if ~reach_goal
      fprintf("RRT fails, return null path \n");
      return;
  end
  disp(['computation time ' num2str(toc)]);
  fprintf("Total iterations =%d \n", total_iter);
  fprintf("number of samples from purely random sampling =%d \n", numFromRand);
  fprintf("number of samples from topological components = %d \n", numFromTopo);
  fprintf("number of purely random samples which are narrow based upon distance checking = %d \n", numNarrowFromRand);
  fprintf("number of topological samples which are narrow based upon distance checking = %d \n", numNarrowFromTopo);
  fprintf("number of samples on Tree = %d \n", numSamplesOnTree);
  fprintf("number of narrow samples on Tree = %d \n", numNarrowOnTree);
  fprintf("number of narrow samples on Tree coming from Topological components =%d \n", numNarrowOnTreeFromTopo);
  

  if (is_benchmark)
    result_dir = 'C:\Users\tingy\OneDrive\matlab_lib\planar_closedchain\';
    dir_name = [result_dir datestr(now, 'yyyy-mm-dd')];
    mkdir(dir_name);
    save([dir_name '/' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.mat'], '-v7.3');
    set(gcf, 'Visible', 'off');
    
    % free memory, sometimes there is a memory leak, or matlab decides to
    % free up memory later.
    clear all;
    clear('rrt.m');
    
    %     problem.plot();
    %     saveas(gcf, [dir_name '\' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.fig']);
  else
    % @narrow_topo is a vector of integers,  
    [path_j, path_c, narrow_topo, smooth_j, smooth_c, path_clearance_raw] = problem.retrieve_path(baseEndPts); 
  end
end