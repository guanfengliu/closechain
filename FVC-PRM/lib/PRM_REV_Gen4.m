function [mpData, path] = PRM_REV_Gen4 (RandomSample, LocalPlanner, ProjectMap, mpData) % k, startc, endc, conf)
k= mpData.conf.neighbors;
startc = mpData.conf.startc;
endc = mpData.conf.endc;
conf = mpData.conf;

%% generation 2 Probabilistic Roadmap Algorithm
%% @outMpData, a data structure that include roadmap sub-structure, obst-region samples, non-obst samples, boundary-variety samples,
% sampling time, and roadmap computation time
%%   RandomSample : A function that returns random samples in freespace, which
% should be replaced by the actual function that implements various
% algorithms for capturing narrow passages in high-dimensional space
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples£»
%   this function may be deprecated by using standard euclidean, or
%   Minkowski metric (in the future maybe just metric type
%
%   LocalPlanner :  A function that determines whether there is a collision-free 
% straight line path between two points in configuration space
%
%   k : The number of neighbors that should be considered in
%        forming the roadmap graph.
%
% Output :
%   outMpData.roadmap - a structure the represent the map, including edges
%   and edge lengths, and vertices
%% Author: Leon G.F. Liu  09/23/2019
%outMpData = struct;
mpData.sampleTime = 0;
mpData.cc = 0; % number of collision checks
%mpData.regular_samples = [];  % number of free samples (include regularly samples
%mpData.obst_samples = []; % number of  samples near C-obstacles (Amato's OBRPM samples) 
mpData.regulars = 0;
mpData.topos = 0;
mpData.near_cobst = 0;
mpData.num_projs = 0;
%outMpData.collision_samples = [];
%mpData.topo_samples = [];  % number of collision samples
mpData.edge_length = []; % roadmap edge length vector
mpData.edges = []; % graph edge map
mpData.total_iters = 0; % total iterations
mpData.nedges = 0;  % total number of edges in the map
%outMpData.nVertices = 0; % total number of vertices
%outMpData.combo_samples = [];  % all free samples
%outMpData.kdtree = [];
path = [];
succ = false;
tic;

extra_samples = [];
while ~succ &&  mpData.total_iters < conf.max_iters, 
  tic;  
  [regular_samples, topo_samples, obst_samples, numCollisionChecks, numIters] = RandomSample();
  mpData.sampleTime = mpData.sampleTime + toc;
  mpData.regulars = mpData.regulars + size(regular_samples, 2);
  mpData.topos = mpData.topos + size(topo_samples, 2);
  
  
  mpData.cc = mpData.cc + numCollisionChecks;
  mpData.total_iters = mpData.total_iters + numIters;
  fprintf (1, 'total iterations %d \n', mpData.total_iters);
 
   tic;
   tmp_samples=[regular_samples, topo_samples, extra_samples]; %samples';
   extra_samples = []; % preparing for next iter
   for i=1:size(tmp_samples,2)
       % find closest neighbors
       new_node = tmp_samples(:,i);
       [nearest_node] = next_node_by_len_close(new_node, mpData);
       num_potential_neighbors = size(nearest_node,2);
       if num_potential_neighbors==0 %% if no nearest node, which means we are just starting ot build the graph
           % we insert the node any way
           [mpData] = insert_node_close([], [], new_node, mpData);
       else
          % we use local planner to connect new node with neighbors
          nearest_neighbors = [];
          dist_neighbors = [];
          for j=1:num_potential_neighbors,
               nbNo = nearest_node(j);
              [out, traj_out, dist, numCCs]=LocalPlanner(new_node, mpData.total_samples(:, nbNo));
              mpData.cc = mpData.cc + numCCs;
              if (out)
                 %outMpData.nVertices = outMpData.nVertices + i;  % a new vertex is generated
                 nearest_neighbors = [nearest_neighbors, nbNo];
                 dist_neighbors = [dist_neighbors, dist];
              else  
                 %if conf.use_fvc  % use_fvc, new algorithm name  
                 % if LocalPlanner fails, then the last milestone which is free will be added to the roadmap 
                 if size(traj_out,2) > 0    %% obstacle-based sampling
                    %[mpData] = insert_node_close(nbNo, dist, traj_out(:,1), mpData);
                    mpData.near_cobst = mpData.near_cobst + 1;
                    extra_samples = [extra_samples, traj_out(:,1)];
                 end
                 if conf.use_fvc
                   %% projecting the collision samples into the cone
                   %% and then reconnect
                   [traj_out, dist, numCCs] = ProjectMap(new_node, mpData.total_samples(:, nbNo));
                   mpData.num_projs = mpData.num_projs + 1;
                   mpData.cc = mpData.cc + numCCs;
                   if size(traj_out,2) > 0
                      %[mpData] = insert_node_close(nbNo, dist, traj_out(:,end), mpData);
                      mpData.topos = mpData.topos + 1;  %size(traj_out,2);
                      extra_samples = [extra_samples, traj_out(:,end)];
                   end
                 end
            %end
             end
          end
          [mpData] = insert_node_close(nearest_neighbors, dist_neighbors, new_node, mpData);
       end
       fprintf (1, 'numVertices=%d, numEdges=%d \n', mpData.nVertices, mpData.nedges);
   end
   
   % reuse all those C-obstacle samples
   for i=1:size(obst_samples,2)
       % find closest neighbors
       new_node = obst_samples(:,i);
       [nearest_node] = next_node_by_len_close(new_node, mpData);
       num_potential_neighbors = size(nearest_node,2);
       for j=1:num_potential_neighbors,
          nbNo = nearest_node(j);
          [out, traj_out, dist, numCCs]=LocalPlanner(new_node, mpData.total_samples(:, nbNo));
          mpData.cc = mpData.cc + numCCs;

          if size(traj_out,2) > 0    %% obstacle-based sampling
              %[mpData] = insert_node_close(nbNo, dist, traj_out(:,1), mpData);
               mpData.near_cobst = mpData.near_cobst + 1;
               extra_samples = [extra_samples, traj_out(:,1)];
          end
          if conf.use_fvc
             %% projecting the collision samples into the cone
             %% and then reconnect
             [traj_out, dist, numCCs] = ProjectMap(new_node, mpData.total_samples(:, nbNo));
             mpData.num_projs = mpData.num_projs + 1;
             mpData.cc = mpData.cc + numCCs;
             if size(traj_out,2) > 0
                %[mpData] = insert_node_close(nbNo, dist, traj_out(:,end), mpData);
                extra_samples = [extra_samples, traj_out(:,end)];
                mpData.topos = mpData.topos + 1;
             end
          end
       end
       fprintf (1, 'numVertices=%d, numEdges=%d \n', mpData.nVertices, mpData.nedges);
   end
     
   
   mapTime=toc;
   mpData.sampleTime = mpData.sampleTime + mapTime;

   tic;
   %% now try to connect startc and endc to the roadmap
   nedges = mpData.nedges;
   edges = mpData.edges;
   edge_lengths = mpData.edge_lengths;
   num_totals = mpData.nVertices;
   num_edges_from_start_rm = 0; % num of edges from start pose to roadmap
   num_edges_from_goal_rm = 0; % num of edges form goal pose to roadmap
 
   [nearest_node] = next_node_by_len_close(startc', mpData);
   num_potential_neighbors = size(nearest_node,2);
   for j=1:num_potential_neighbors,
        nbNo = nearest_node(j);
        [out, traj_out, dist, numCCs]=LocalPlanner(startc', mpData.total_samples(:, nbNo));
        mpData.cc = mpData.cc + numCCs;
        if out
           nedges = nedges + 1;
           edges(nedges,:) = [num_totals+1, nbNo];
           edge_lengths(nedges) = dist;
           num_edges_from_start_rm =  num_edges_from_start_rm + 1;
        else
            if size(traj_out,2) > 0
               mpData.near_cobst = mpData.near_cobst + 1;
               extra_samples = [extra_samples, traj_out(:,1)];
            end
        end
   end
  
 
   fprintf (1, 'connecting start to roadmap: nedges = %d\n', num_edges_from_start_rm);
   [nearest_node] = next_node_by_len_close(endc', mpData);
   num_potential_neighbors = size(nearest_node,2);
   for j=1:num_potential_neighbors,
        nbNo = nearest_node(j);
        [out, traj_out, dist, numCCs]=LocalPlanner(endc', mpData.total_samples(:, nbNo));
        mpData.cc = mpData.cc + numCCs;
        if out
           nedges = nedges + 1;
           edges(nedges,:) = [num_totals+2, nbNo];
           edge_lengths(nedges) = dist;
           num_edges_from_start_rm =  num_edges_from_start_rm + 1;
        else
            if size(traj_out,2) > 0
               mpData.near_cobst = mpData.near_cobst + 1;
               extra_samples = [extra_samples, traj_out(:,1)];
            end
        end
   end
   
   fprintf (1, 'connecting goal to roadmap:  nedges = %d\n', num_edges_from_goal_rm);
   % if both start and goal can be connected to roadmap, check if they are
   % connected in the roadmap
   if num_edges_from_start_rm > 0 && num_edges_from_goal_rm > 0
        SS= edges(:,1)';
        TT= edges(:,2)';
        weights= edge_lengths';
        G = graph(SS,TT,weights);
        numComp = max(conncomp(G));
        fprintf(1,'numCom=%d \n', numComp);
        route = shortestpath(G,num_totals+1, num_totals+2);
        mapTime = toc;
        mpData.sampleTime = mpData.sampleTime + mapTime;
        if length(route) > 0
            path = mpData.total_samples(:,route(2:end-1));
            path = [startc, path, endc];
            succ = true;
           
            % drawAndSavePathGen4(mpData, path, start_fig_id, startc, endc);
        end
   end
   fprintf("Total iterations =%d \n", mpData.total_iters);
end
disp(['computation time ' num2str(mpData.sampleTime)]); 
fprintf('Total iterations =%d \n', mpData.total_iters);
fprintf('Number of ccs =%d \n', mpData.cc);
fprintf('Number of regular samples =%d \n', mpData.regulars);
fprintf('Number of topo samples =%d \n', mpData.topos);
fprintf('Number of near_obst samples =%d \n', mpData.near_cobst);
fprintf('Number of fvc projections =%d \n', mpData.near_projs);

end