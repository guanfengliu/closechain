classdef FNClimbRobot_CONNECT < handle
    properties (SetAccess = private)
        tree                    % Tree from start point, Array stores states 
        parent                  % Array stores relation information about nodes                    
        children                % Number of children of each node
        
        %cost                    % Cost between 2 connected states
        %cumcost                 % Cost from the root of the tree to the given node
        conf                    % configuration struct
        XY_BOUNDARY             %[min_x max_x min_y max_y]
        start_hold_index           % start hold No 
        goal_hold_index            % goal hold No
        %num_holds               % number of holds
        holds                   % a structure, holds.num  -- number of holds
                                %              holds.coord  --  2 by n
                                %              matrix of hold coordinates
        holdGraph               % the adjacency graph among holds
        paths                   % paths{i}.hold_pair ={hold1, hold2}
                                % paths{i}.path = {list of points between
                                % hold1 and hold2}
        goal_point              % tip coordinates of the chain at global goal configuration
        start_point             % tip coordinates of the chain at global start configuration
        goal_angle              % chain joint angles of global goal configuration
        start_angle             % chain joint angles of global start configuration
        delta_goal_point        % radius of goal position
        delta_ang_max           % maximal change in angle for each of the joints while adding new node
        delta_ang_neighbor      % maximal change in angle for each of the joints while seeking for neighbors
        nodes_added             % the length of tree
        max_ang                 % maximum angle a joint can go
        min_ang                 % minimum angle a joint can go
        num_links                % number of links of Redudant Manipulator (DOF)
        len_link                % length of each of link
        width_link              % width of each of link
        height_link             % height of each link
        minimalCritRadius       % minimal critical radius
        maximalCritRadius       % maximal critical radius
        obstacle                % obstacle information
        narrow_vertices         % each row is a vertex index which in one-2-one correpondance with the row of faces
        narrow_faces            % each row represent edges from a different obstacle
        num_narrow_pairs        % num of narrow vertex-edge pairs for sampling narrow passages
        %narrowLinkSet           % the set of narrow links 
        best_path_node 
        goal_reached 
        % maximal number of nodes
        max_nodes
        
        
        %%% temporary variables
        temp_new_node_position
        
        %%% obstacle detection specific
        turn_states             % intermediate angle states from one configuration of redudant robot manipulator to another
%         turn_pos                % intermediate position states from one configuration of redudant robot manipulator to another
%         turn_cumsum             % intermediate states, cummulative sum
        test_list               % combination of links for self collision check up
        num_comb                % number of combinations
%         link_cen                % positions of centers of links
%         link_corner             % OBB(Oriented Bounding Boxes) of each links
%         link_axis               % 
%         link_quant              % Link quantization
%         obs_mat                 % special matrix for obstacle collision
        step_div                % number of intermediate step between two states
        temp_obs                
    end
    methods
        % object constructor
        % load_map(map.name)  loads variables like obstacles (n polygons, each a 2 by n matrix), 
        % holds (2*n matrix), 
        function this = FNClimbRobot_CONNECT(rand_seed, max_nodes, map, conf)
            rng(rand_seed);
            %% initialize configuration parameters
            this.conf = conf;
            %% initialize narrow vertices and edge pairs used in topological component sampling
            this.narrow_vertices = [];
            this.narrow_faces = [];
            this.num_narrow_pairs = 0;
            this.max_nodes = max_nodes;
            
            %% start and goal hold id
            this.start_hold_index = map.start_hold_index;
            this.goal_hold_index = map.goal_hold_index;
            %this.min_ang = conf.min_ang;
            %this.max_ang = conf.max_ang;
            %this.len_link = conf.len_link;
            this.num_links = conf.num_links;
            %this.width_link = conf.width_link;
            %this.height_link = conf.height_link;
             
            %% compute the maximal and minimal critical radius
            this.computeCritRadius(this.conf.len_link);
            %% load hold map and obstacle map
            this.load_map(map.name);  %% load holds, obstacles, etc.
            %this.num_holds = this.holds.num;
            %this.conf.neighbors denots the number of closest neighbors
            %used in knn search
            this.holdGraph = this.buildHoldGraph();
            %% search through obstacles, and then find narrow face-vertex pairs
            this.findNarrowFaceVertexPairs();
            this.tree = struct;
            this.tree.mpData = struct;
            % tree is for RRT algorithm, it will be rebuilt for every
            % phase: e.g. phase a: from start to graph a hold, phase b:
            % adjust configuration to the correct configuration when both
            % ends of the arm grabs holds; phase 3: break one hold and
            % reach the next hold
            this.tree.angle = zeros(this.conf.num_links, max_nodes, 'single');
            % a vector of integers, 0 means not narrow, 1 means narrow and
            % also coming from topological components
            this.tree.narrow_topo = zeros(1, max_nodes,'single');
            % n link has n+1 joint points
            this.tree.position = zeros(this.conf.num_links+1, 2, max_nodes, 'single');
            % tree.base means for this RRT, all robot configurations has
            % the same base, given by tree.base
            this.tree.base = this.holds.coord(:, this.start_hold_index);
            % goal angle is mainly for adjusting the robot config to right
            % configuration at the right hold point
            this.tree.goal_angle = this.conf.goal_conf;
            %@mode, "close chain"  then in closed chain mode
            %   ="open chain" then in open chain mode
            % @terminal = 0, using link 0 (or hold) as terminal, =1, using link
            % n+1 as terminal (also a hold)
            % for open chain hold_id_l == hold_id_r, robot is based at
            % hold_id1(hold_id2)
            this.tree.mode = "open";
            this.tree.hold_id_l = this.start_hold_index;
            % the following hold_id_r should be changed based upon actual
            % hold sequence during climbing
            this.tree.hold_id_r = this.goal_hold_index; % this.start_hold_index;
            this.tree.terminal = 0;  %% based on left
            % tree.linkLengthVec means for this RRT, all robot
            % configuration has the same linklength vector, given by
            % tree.linkLengthVec
            if ~this.tree.terminal
                this.tree.linkLengthVec = this.conf.len_link;
                this.tree.linkWidthVec = this.conf.width_link;
                this.tree.linkHeightVec = this.conf.height_link;
                this.tree.max_ang = this.conf.max_ang;  %% this will change when different end of chain holds the hold
                this.tree.min_ang = this.conf.min_ang;
            else
                this.tree.LinkLengthVec = this.conf.len_link(end:-1:1);
                this.tree.linkWidthVec = this.conf.width_link(end:-1:1);
                this.tree.linkHeightVec = this.conf.height_link(end:-1:1);
                this.tree.max_ang = this.conf.max_ang(end:-1:1);  %% this will change when different end of chain holds the hold
                this.tree.min_ang = this.conf.min_ang(end:-1:1);
            end
            this.temp_new_node_position = zeros(conf.num_links, 2, 'single');
                
            this.parent = zeros(1, max_nodes, 'int32');
            this.children = zeros(1, max_nodes, 'int32');
            this.start_angle = conf.init_conf;
            this.goal_angle = conf.goal_conf;
            this.tree.angle(:, 1) = conf.init_conf;
            this.tree.narrow_topo(1) = 0;  %initial angle is always set to non-narrow nor from topological component
                      
            % tip coordinates of the start and goal configuration (goal tip
            % will be changed base upon the eventual link id at goal hold
            % point
            this.start_point(1) = sum( this.tree.linkLengthVec.*cos(this.start_angle)) + this.tree.base(1);
            this.start_point(2) = sum( this.tree.linkLengthVec.*sin(this.start_angle)) + this.tree.base(2);
            this.goal_point(1) = sum( this.tree.linkLengthVec.*cos(this.goal_angle)) + this.tree.base(1);
            this.goal_point(2) = sum( this.tree.linkLengthVec.*sin(this.goal_angle)) + this.tree.base(2);
            
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_ang_max = conf.delta_ang_max;
            this.delta_ang_neighbor = conf.delta_ang_neighbor;
            this.nodes_added = 2;
            
            
            %%% obstacle detection specific: the number of interpolation
            %%% points for collision checking
            this.step_div = conf.step_div;
            
            % for link-link collision checking
            this.test_list = combnk(1:this.num_links,2);                               % 2uple combinations of 5 links
            this.test_list = setdiff(this.test_list,  [ (1:this.num_links-1)' (2:this.num_links)'], 'rows');    % Remove adjacent links
            [this.num_comb, ~] = size(this.test_list);
        end
        %% Access some properties
        function hold_id = getStartHoldid(this)
            hold_id = this.start_hold_index;
        end
        function hold_id = getGoalHoldid(this)
            hold_id = this.goal_hold_index;
        end
        function delta_angle_norm = getNeighborDiff(this)
            delta_angle_norm = this.delta_ang_neighbor;
        end
        %% get start and goal config
        function [start_j, start_c] = getStartConfig(this)
            start_j = this.start_angle; %this.conf.init_conf;
            start_c = this.generate_link_position(start_j);
            return;
        end
        function [desired_j] = getDesiredConfig(this)
            desired_j = this.conf.goal_conf;
            return;
        end
        %% update tree
        function status = updateTree(this, hold_left, hold_right, terminal, mode, start_j, desired_j) 
            %% update left and right holds
            this.tree.hold_id_l = hold_left;
            this.tree.hold_id_r = hold_right;
            %% update base
            this.tree.base = this.holds.coord(:, this.tree.hold_id_l);
            %% update terminal (robot kinematics based upon which base link)
            this.tree.terminal = terminal;
            if ~terminal
                this.tree.linkLengthVec = this.conf.len_link;
                this.tree.linkWidthVec = this.conf.width_link;
                this.tree.linkHeightVec = this.conf.height_link;
                this.tree.max_ang = this.conf.max_ang;  %% this will change when different end of chain holds the hold
                this.tree.min_ang = this.conf.min_ang;
            else
                this.tree.LinkLengthVec = this.conf.len_link(end:-1:1);
                this.tree.linkWidthVec = this.conf.width_link(end:-1:1);
                this.tree.linkHeightVec = this.conf.height_link(end:-1:1);
                this.tree.max_ang = this.conf.max_ang(end:-1:1);  %% this will change when different end of chain holds the hold
                this.tree.min_ang = this.conf.min_ang(end:-1:1);
            end
            %% update tree mode (open or closed chain)
            this.tree.mode = mode;
            % reinit this.tree.angle
            this.tree.angle = zeros(this.conf.num_links, this.max_nodes, 'single');
            % reinit temp variable
            this.temp_new_node_position = zeros(this.conf.num_links, 2, 'single');
            % reinit parent and children    
            this.parent = zeros(1, this.max_nodes, 'int32');
            this.children = zeros(1, this.max_nodes, 'int32');
            
            % reinit cost and cumcost
            %this.cost = zeros(1, this.max_nodes, 'single');
            %this.cumcost = zeros(1, this.max_nodes, 'single');
            
            % update first node angle on the tee
            this.tree.angle(:, 1) = start_j;
            this.tree.narrow_topo(1) = 0;  %initial angle is always set to non-narrow nor from topological component
            % update the link positions of first node on the tree
            this.update_link_position(1);
            % reinit number of nodes added
            this.nodes_added = 2;
            % reinit turn states
            this.turn_states = zeros(this.num_links, this.step_div, 'single'); % angles
            
            
            %% desired joint only makes sense for the very last motion at the goal hold
            if length(desired_j) <= 1
               dvec = this.holds.coord(:, this.tree.hold_id_r) - this.tree.base;
               gap = norm(dvec);
                  
               if mode == "closed"   % for closed chain, we need cl_center and cl_radius to estimate workspace
                  this.tree.cl_center =  0.5 * (this.holds.coord(:, this.tree.hold_id_r) + this.tree.base);
                  if this.conf.half_total_length > 0.5 * gap
                     this.tree.cl_radius = (this.conf.half_total_length^2 - (0.5*gap)^2)^0.5; 
                  else
                     this.tree.cl_radius = -1;
                  end
               end
               
               this.tree.mpData.linklength = [this.tree.linkLengthVec; gap]; 
               [CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(this.tree.mpData.linklength);
               this.tree.mpData.init_angle = atan2(dvec(2), dvec(1));
               this.tree.mpData.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
               this.tree.mpData.CollisionVarCritRadiusR = CollisionVarCritRadiusR;
               % @enable_bound, do we also smaple the boundary variety
               this.tree.mpData.enable_bound = false;
               this.tree.mpData.left_2_right = true;
               this.tree.mpData.nonbsamples = this.conf.init_search_tries;
                  
               collision = true;
               numTries = 0;
               while collision && numTries < 5 * this.conf.init_search_tries
                  [out] = RandomLoopGeneratorGen2(this.tree.mpData);
                  if size(out,2)==0
                     status=false;
                     return;
                  else
                     for i=1:size(out,2) 
                       angle = out(1:end-1,i);
                       collision = this.CheckCollision(angle);
                       if ~collision
                           break;
                       end
                     end
                     %this.tree.goal_angle(1:this.num_links-1) = out(1:end-1,1);
                  end
                  numTries = numTries + size(out,2);
               end
               if collision %numTries >= this.conf.init_search_tries
                  status = false; %% search valid init failed.
                  return;
               end
               this.tree.goal_angle = angle;
            else
               this.tree.goal_angle =  desired_j; % * ones(this.conf.num_link,1);
            end
            this.goal_point(1) = sum( this.tree.linkLengthVec.*cos(this.tree.goal_angle)) + this.tree.base(1);
            this.goal_point(2) = sum( this.tree.linkLengthVec.*sin(this.tree.goal_angle)) + this.tree.base(2);
            status=true;
            return;
        end
        % get the goal angle for this current phase of motion
        function goal_angle = getCurrentGoal(this)
             goal_angle = this.tree.goal_angle;
        end
        %% process all obstacles, and create their triangulation representation
        function   processObstacle(this)
              this.obstacle.fv.vertices = [];
              this.obstacle.fv.faces = [];
              % obstacle triangulation
              for i=1:this.obstacle.numObst
                  this.obstacle.fv = appendFV(this.obstacle.fv, TriangPolygon(this.obstacle.obstDesc{i}));
              end
        end
        %% compute the tangent and normal of obstacle edges
        function [tl, tr, nl, nr] = computeTangentAndNormal(this, node_l, node_c, node_r, obst_id)
            pl = this.obstacle.coord(:, node_l);
            pc = this.obstacle.coord(:, node_c);
            pr = this.obstacle.coord(:, node_r);
            tl = (pc - pl) / norm(pc - pl);
            tr = (pr - pc) / norm(pr - pc);
            dd = pc - this.obstacle.center{obst_id};
            nl = dd - (dd' * tl) * tl;
            nl = nl/norm(nl);
               
            nr = dd - (dd' * tr) * tr;
            nr = nr/norm(nr);
        end
        %% check if diffVec has <90 degree from the normal_vec
        % @normal_vec is a unit normal vector
        % @diff_vec might not be a unit vector
        function positive = checkNormalDirection(this, normal_vec, diff_vec)
            val = normal_vec' * (diff_vec/norm(diff_vec));
            if val > this.conf.half_plane_epsilon
                positive = true;
            else
                positive = false;
            end
        end
        %% calculate the minimal distance between qq and the line segment pp1->pp2
        function [dist, closestPt] = computeDistVertex2Edge(this, qq, pp1, pp2)
            dv = pp2 - pp1;
            dq = pp1 - qq;
            t = - (dq' * dv)/((norm(dv))^2);
            if t >= 0 && t <= 1
                dist = abs(dv(1) * dq(2) - dv(2) * dq(1))/norm(dv);
                closestPt = pp1 + t * dv;
            else
                if norm(qq-pp1) < norm(qq-pp2)
                  dist = norm(qq-pp1);
                  closestPt = pp1;
                else
                  dist = norm(qq-pp2);
                  closestPt = pp2;
                end
               % dist = min(norm(qq - pp1), norm(qq-pp2));
            end
            % subtract the half of the link wdith
            dist = dist - 0.5 * this.conf.width;
        end
        %% find narrow face-vertex pairs 
        function findNarrowFaceVertexPairs(this)
            % vertice matrix of the vertices from all obstacles, rows are coordinates of vertices
            this.obstacle.coord = [];
            tempArray = [];
            total_vertices = 0;
            % next 
             for i=1:this.obstacle.numObst 
                this.obstacle.coord = [this.obstacle.coord, this.obstacle.obstDesc{i}];
                tempArray = [tempArray, size(this.obstacle.obstDesc{i}, 2)];
            end
            total_vertices = sum(tempArray);
            vertArray = cumsum(tempArray);
            % next is a cell array for computing the adjacency vertices
            next = cell(total_vertices, 1);
            % compute the normal and tangent of the edges containing a given vertex
            normal_next = cell(total_vertices, 1);
            tangent_next = cell(total_vertices, 1);
            vert_offset = 0;
            for i=1:this.obstacle.numObst,
               node_l = vert_offset + tempArray(i);
               node_c = vert_offset + 1;
               node_r = vert_offset + 2;
               next{node_c} = [node_l, node_r];
               [tl, tr, nl, nr] = this.computeTangentAndNormal(node_l, node_c, node_r, i);
               %% tangents and normals of the two edges adjcent to node_c
               tangent_next{node_c} = [tl, tr];
               normal_next{node_c} = [nl, nr];
               for j=2:tempArray(i)-1,
                   %next{vert_offset+j} = [vert_offset+j-1, vert_offset+j+1];
                   node_l = vert_offset+j-1;
                   node_c = vert_offset+j;
                   node_r = vert_offset+j+1;
                   next{node_c} = [node_l, node_r];
                   [tl, tr, nl, nr] = this.computeTangentAndNormal(node_l, node_c, node_r, i);
                   %% tangents and normals of the two edges adjcent to node_c
                   tangent_next{node_c} = [tl, tr];
                   normal_next{node_c} = [nl, nr];
               end
               node_l = vertArray(i)-1;
               node_c = vertArray(i);
               node_r = vert_offset+1;
               next{vertArray(i)} = [node_l, node_r];
               [tl, tr, nl, nr] = this.computeTangentAndNormal(node_l, node_c, node_r, i);
               %% tangents and normals of the two edges adjcent to node_c
               tangent_next{node_c} = [tl, tr];
               normal_next{node_c} = [nl, nr];
               vert_offset = vertArray(i);
            end
            
            %% using knn search to find the cloest neighbors
            %%kdtree =createns(X,'NSMethod','exhaustive','Distance',@DistClosedChainKnn);  %%
            kdtree= KDTreeSearcher(this.obstacle.coord','Distance','minkowski','BucketSize', 2 * this.conf.neighbors);  %% we make sure the bucket size is a little bit high than k 

            %% find the closest k+1 neighbor for each point in X 
            IdxNN = knnsearch(kdtree, this.obstacle.coord', 'K', this.conf.neighbors + 1);
            sz_IdxNN = size(IdxNN);
            sz_r = sz_IdxNN(1);
            sz_c = sz_IdxNN(2);
            %% interval_index determines which obtacle a vertex belong to
            interval_index = 1;
            left_index = 0;
            right_index = vertArray(interval_index);
            for i = 1:sz_r,
               if i > right_index
                   interval_index = interval_index + 1;
                   left_index = right_index;
                   right_index = vertArray(interval_index);
               end
               yy = this.obstacle.coord(:, i);  %% target point
               % check all k nearest neighbors, see if they can be reached
               % from y
               for j=2:sz_c,  %% recall j=1 is the y itself   
                  %%  if IdxNN(i,j) is not in the same polygon 
                  indx = IdxNN(i,j);
                  if indx <= left_index || indx > right_index
                     % find out the edges that include the vertex indx
                     next_l = next{indx}(1);
                     pc = this.obstacle.coord(:, indx);
                     pl = this.obstacle.coord(:, next_l);
                     dist = this.computeDistVertex2Edge(yy, pl, pc);
                     if dist <= this.conf.narrow_dist  %% n
                         % narrow faces, but need to check if the vertices
                         % lies in the positive half plane of the narrow
                         % edge
                         positive = this.checkNormalDirection(normal_next{indx}(:,1), yy-pc);
                         if positive
                            this.narrow_faces = [this.narrow_faces; indx, next_l];
                            % narrow vertices
                            this.narrow_vertices=[this.narrow_vertices;i];
                         end
                     end
                     next_r = next{indx}(2);
                     pr = this.obstacle.coord(:, next_r);
                     dist = this.computeDistVertex2Edge(yy, pr, pc);
                     if dist <= this.conf.narrow_dist  %% n
                         positive = this.checkNormalDirection(normal_next{indx}(:,2), yy-pc);
                         if positive
                            this.narrow_faces = [this.narrow_faces; indx, next_r];
                            % narrow vertices
                            this.narrow_vertices=[this.narrow_vertices;i];
                         end
                     end
                  end
               end
            end
           this.num_narrow_pairs = length(this.narrow_vertices);
           fprintf (1, 'num narrow vertex-edge pair %d\n', this.num_narrow_pairs); 
        end
        
        function computeCritRadius(this, linklength)
            this.minimalCritRadius = 0;
            this.maximalCritRadius = 0;
            max_link_length = max(linklength);
            sum_link_length = sum(linklength);
            if max_link_length > 0.5 * sum_link_length
                error("We require the workspace of robot being a disk containing the robot origin");
            else
                this.maximalCritRadius = sum_link_length;
            end
        end
        %% compute the minimal clearance between the links of a robot and the obstacles
        % @joint, the input joint angle for which we want to determine its
        % min-clearance (Cartesian) from the obstacles,
        % @narrow_dimension, determine if a link lies in a narrow passage,
        % and whose dimension of narrowness
        function [minDist] = calMinClearance(this, joint)
            link_pos = this.generate_link_position(joint);
            minDist = inf;
            %isNarrow = false;
            for i=1:this.num_links,
                dist =  this.calObstDist(link_pos(i,:)', link_pos(i+1,:)');
                if dist < minDist
                    minDist = dist;
                end
            end
        end
        
        function [minDist, isNarrow] = calNarrowNess(this, joint)
            link_pos = this.generate_link_position(joint);
            minDist = inf;
            isNarrow = false;
            for i=1:this.num_links,
                [dist, isNarrowTemp] = this.determineNarrowDist(link_pos(i,:)', link_pos(i+1,:)');
                if ~isNarrow && isNarrowTemp
                    isNarrow = true;
                end
                if dist < minDist
                    minDist = dist;
                end
            end
        end
        
        
        % goes through all vertices, and compute the minimal distance, and
        % then if the closest vertex belong to a narrow vertex index, and 
        % then if the link parallel to the narrow edge, then report that
        % this configuration belong to the narrow configuration
        function  [dist] = calObstDist(this, pp1, pp2)
            dist = inf;
            for i=1:size(this.obstacle.coord, 2),
                qq = this.obstacle.coord(:, i);
                dist_t = abs(this.computeDistVertex2Edge(qq, pp1, pp2));
                if dist_t < dist
                   dist = dist_t;
                end
            end
        end
        
        %% given a triangle p1-p2-p3, and if the current chain forms a closed chain, find all
        %% edges (and closest points on them) such that the gap between edge and this.tree.cl_center is 
        %% less than this.tree.cl_radius.  tree.cl_center and tree.cl_radius are estimate of the workspace
        %% of link 3 or link 4 so that (link1..link3) and (link4,link5,link6) both forms a valid closed chain
        function [opposite_indices, closestPts] = findTriangleEdgesWithinWorkSpace(this, p1, p2, p3)
            opposite_indices = [];
            closestPts = [];
            minDist = this.tree.cl_radius;
            minq = [];
            minOppInd = [];
            [dist_1, q1] = this.computeDistVertex2Edge(this.tree.cl_center, p1, p2);
            if dist_1 < minDist
               minDist = dist_1;
               minq = q1;
               minOppInd = 3;
            end
            
            [dist_2, q2] = this.computeDistVertex2Edge(this.tree.cl_center, p2, p3);
            if dist_2 < minDist
               minDist = dist_2;
               minq = q2;
               minOppInd = 1;
            end
            
            [dist_3, q3] = this.computeDistVertex2Edge(this.tree.cl_center, p3, p1);
            if dist_3 < minDist
                minDist = dist_3;
                minq = q3;
                minOppInd = 2;
            end
            opposite_indices = [opposite_indices, minOppInd];
            closestPts = [closestPts, minq];
        end
        
        % goes through all narrow vertex-edge pairs, and compute the actual
        % minimal among max-gaps between link and each narrow pair, and determine if it is narrow
        function [dist, isNarrow] = determineNarrowDist(this, pp1, pp2) 
            numNarrowVertex = length(this.narrow_vertices);
            % check every narrow vertex-face pairs for possible closeness
            dist = inf;
            isNarrow = false;
            for i=1:numNarrowVertex,
                qq = this.obstacle.coord(:, this.narrow_vertices(i));
                narrow_edge = this.narrow_faces(i,:);
                qq1 = this.obstacle.coord(:, narrow_edge(1));
                qq2 = this.obstacle.coord(:, narrow_edge(2));
                
                dist_qq = this.computeDistVertex2Edge(qq, pp1, pp2);
                dist_21 = this.computeDistVertex2Edge(qq1, pp1, pp2);
                dist_22 = this.computeDistVertex2Edge(qq2, pp1, pp2);
                dist_23 = this.computeDistVertex2Edge(pp1, qq1, qq2);
                dist_24 = this.computeDistVertex2Edge(pp2, qq1, qq2);
                dist_qq12 = min([dist_21,dist_22,dist_23,dist_24]);
                
                % also need to check if qq and qq_1 (or qq_2) lies in the
                % two side of pp1-pp2 line segment
                norm_p12 = [pp1(2)-pp2(2), pp2(1) - pp1(1)]; %row vector
                vv1 = norm_p12 * (qq-pp1);
                vv2 = norm_p12 * (qq1-pp1);
                vv3 = norm_p12 * (qq2-pp1);
                in_opsite = false;
                if vv1 * vv2 < 0 && vv1 * vv3 < 0
                    in_opsite = true;
                end
                
                dist_t = max(dist_qq, dist_qq12); %min(dist_qq, dist_qq12);
                if dist_t < dist,
                    dist =  dist_t;
                end
                if dist_qq < this.conf.narrow_link_dist  && dist_qq12 < this.conf.narrow_link_dist && in_opsite && ~isNarrow
                    isNarrow = true;
                end
            end
        end
        
        %% build hold graph for searching shortesting hold sequence in climbing
        function hGraph = buildHoldGraph(this)
           hGraph = graph();
           %build graph from holds: holds.coord is a 2 by k matrix,  holds.num is the number of holds
           %we apply knn search algorithm to build the graph
           %% create kd-tree for later on applying nearest neighbor search
           X=this.holds.coord'; %% each row of X is the coordinates of a hold
           %%kdtree =createns(X,'NSMethod','exhaustive','Distance',@DistClosedChainKnn);  %%
           kdtree= KDTreeSearcher(X,'Distance','minkowski','BucketSize', 2 * this.conf.neighbors);  %% we make sure the bucket size is a little bit high than k 

           %% find the closest k+1 neighbor for each point in X 
           IdxNN = knnsearch(kdtree, X, 'K', this.conf.neighbors + 1);
           sz_IdxNN = size(IdxNN);
           sz_r = sz_IdxNN(1);
           sz_c = sz_IdxNN(2);
           for i = 1:sz_r,
               y = this.holds.coord(:,i);  %% target point
               % check all k nearest neighbors, see if they can be reached
               % from y
               for j=2:sz_c,  %% recall j=1 is the y itself   
                  x =  this.holds.coord(:,IdxNN(i,j)); 
                  dist = norm(x-y);
                  %% here 3 conditions, first 
                  if dist <= 0.75 * this.maximalCritRadius  && dist >= this.minimalCritRadius && x(2) >= y(2)
                    hGraph = addedge(hGraph, i, IdxNN(i,j), dist);
                  end
               end
           end
           fprintf (1, 'numVertices = %d, nedges = %d\n', this.holds.num, numedges(hGraph)); 
        end
        
        %% search viable hold sequence gPath is a list of possible paths based upon "shortest path"  heuristics
        function gPath = searchHoldSequence(this)
            %% first compute the adjacency matrix adj
            adj = adjacency(this.holdGraph);
            gPath = paths_climb(adj, this.start_hold_index,...
                  this.goal_hold_index, this.holds.coord, false);
        end
        %% sampling based upon robot current phase, sometimes, robot is
        %% in close chain mode, and otherwise, it may be an open chain 
        %% based at link 1 or link n (last link)
        %@mode, "close chain"  then in closed chain mode
        %   ="open chain" then in open chain mode
        % @terminal = 0, using link 0 (or hold) as terminal, =1, using link
        % n+1 as terminal (also a hold)
        % for open chain hold_id_l == hold_id_r, robot is based at
        % hold_id1(hold_id2)
        function [state, numCollisionChecks] = sample(this) % mode, hold_id_l, hold_id_r, terminal)
            state = [];
            numCollisionChecks = 0;
            % generate random angle within min_ang and max_ang range
            % state represents  a C-space element, which consists of an
            % joint angle vector and the 2d coordinates of base
            if this.tree.mode == "open"
               state = (this.tree.max_ang - this.tree.min_ang) .* rand(this.num_links, 1) + this.tree.min_ang;
            else
               [out] = RandomLoopGeneratorGen2(this.tree.mpData);
               if size(out,2)==0
                  return;
               else
                  state = out(1:end-1, 1);
               end
            end
            collision = this.CheckCollision(state);
            numCollisionChecks = numCollisionChecks + 1;
            if collision
               state = [];
            end
        end
        
        %% randomly choose the set of narrow links and the corresponding narrow vertex-edge pairs, for which the entire chain
        % can be closed by inverse kinematics
        function [narrowLinkSet, narrowPassageSet] = computeNarrowLinkSet(this) %, mode)  %, terminal)
          if this.tree.mode=="closed"  % closed chain
              minNarrowLink = 3;
              maxNarrowLink = this.num_links - 2;
          else   % open chain
%             if terminal   %based at n+1 
%                 %% choose (k1, k2, k3, ..., km) that lies in narrow
%                 % passages, km <= n-2
%                 maxNarrowLink = this.num_links - 2;  % maximal number of narrow links
%                 minNarrowLink = 1;
%             else
%                 maxNarrowLink = this.num_links;
%                 minNarrowLink = 3;
%             end
              %% chain angles have different definition for open chains with different base links
              %% but they are always numbered as 1 to n starting from the base, for computation conveninence
              minNarrowLink = 3;
              maxNarrowLink = this.num_links;
          end
          
          % first narrow link
          startNarrowLink = randi([minNarrowLink, maxNarrowLink], 1);
          narrowLinkSet = [startNarrowLink];
          while narrowLinkSet(end) + 3 <= maxNarrowLink 
            nextlink = randi([narrowLinkSet(end)+3, maxNarrowLink],1);
            narrowLinkSet = [narrowLinkSet, nextlink];
          end
          num_narrow_links = length(narrowLinkSet);
          %% for each narrow link chose correspongding narrow vertex_edge pairs
          narrowPassageSet = randi(this.num_narrow_pairs, num_narrow_links, 1);
        end
        
        
        %% sampling open chain topological components
        %@terminal =0, link 0 as base of open chain, terminal=1, link n+1
        %as base of open chain
        % @mode ="open" is open chain, ="closed" is closed chain
        % for open chain, hold_id_l=hold_id_r,  for closed chain, hold_id_l
        % is the terminal link (indicated by terminal), hold_id_r is the other terminal link
        
        function [topo_state, topo_state_c, numCollisionChecks] = sampleTopo(this) % mode, hold_id_l, hold_id_r, terminal)
           topo_state = []; % joint state
           topo_state_c = []; % Cartesian state
           numCollisionChecks = 0;
           if this.num_narrow_pairs > 0
               % compute the vector of narrow links
               % and the cooresponding index vector, representing the set of pairs of narrow vertex-edge pairs
              [narrowLinkSet, narrowPairSet] = this.computeNarrowLinkSet();  
              num_narrow_links = length(narrowLinkSet);
              base_terminal = this.tree.base; %top_state.base;
              base_link = 1;
              if num_narrow_links > 0
                 for i=1:num_narrow_links
                    % which narrow vertex-edge pair
                    narrowPair = narrowPairSet(i);
                    narrowEdge = this.narrow_faces(narrowPair, :);
                    narrowVertex = this.narrow_vertices(narrowPair);
                    % pt(1:2,:) is the narrowEdge
                    pt = this.obstacle.coord(:, narrowEdge);
                    % pt(3,:) is the narrow vertex
                    pt = [pt, this.obstacle.coord(:, narrowVertex)];
                    % compute the distances from all three points of a
                    % triangle to this.tree.cl_center, and compared them
                    % with this.tree.cl_rdius, return indices of those
                    % within 
                    if this.tree.mode ~="open"
                      % compute a point on the triangle which is closest to the middle point of the two anchors
                      % and its opposite vertex, this will make loop
                      % closure constraints easier to satisfied for both
                      % two closed chains formed by fixing a narrow link
                      % pose
                      [opposite_indices, closestPts] = this.findTriangleEdgesWithinWorkSpace(pt(:,1), pt(:,2), pt(:,3));
                      if length(opposite_indices) == 0
                         topo_state = []; % joint state
                         topo_state_c = []; % Cartesian state
                         return;
                      end
                      % let centerPt be closer to closestPts;
                      pp_up = 0.5 * (pt(:, opposite_indices(1)) + closestPts);
                      ra = rand(1);
                      centerPt =  ra * pp_up + (1-ra) * closestPts;
                    else
                      % for open chain, just directly purely random sampling  
                      % randNumber = rand(3, 1);
                       pt_l = 0.5 * (pt(:, 3) + pt(:, 1));
                       pt_r = 0.5 * (pt(:, 3) + pt(:, 2));
                       centerPt = pt_l + rand(1) * (pt_r - pt_l);  %pt * (randNumber/sum(randNumber));
                    end
                    
                   
                    
                    dpt = pt(:, 2) - pt(:, 1);
                    % this is median axis direction for avoiding collision
                    angle = atan2(dpt(2), dpt(1));
                    
                    %% narrow link index  n_link
                    n_link = narrowLinkSet(i);
                    % randomly chosen the fraction of the link n_link that
                    % points from centerPt to pt_r
                    frac = rand(1);
                    linkVec = this.tree.linkLengthVec(n_link) * [cos(angle); sin(angle)];
                    % randomly choose directions 
                    dirRand = rand(1);
                    if dirRand > 0.5  %  frac *  linkVec  points toward pt_r,  (1-frac) * linkVec pointers from center towrad pt_l
                       narrow_endPts = centerPt + [frac * linkVec,-(1-frac) * linkVec] ;
                    else
                       narrow_endPts = centerPt - [frac * linkVec,-(1-frac) * linkVec] ;
                    end
                    %% now create open or closed chains to apply IK for sampling their configurations.
                    dvec = narrow_endPts(:, 1) - base_terminal;
                    mpData.linklength = [this.tree.linkLengthVec(base_link:n_link-1); norm(dvec)]';
                    base_link = n_link + 1;
                    base_terminal = narrow_endPts(:, 2);
%                     if terminal  %% base at link n+1
%                        dvec = r_terminal - narrow_endPts(2,:);
%                        mpData.linklength = this.conf.len_link(n_link+1, r_link);
%                        r_link = n_link-1;
%                        r_terminal = narrow_endPts(1,:);
%                     else 
%                        dvec = narrow_endPts(1,:) - l_terminal;
%                        mpData.linklength = this.conf.len_link(l_link, n_link - 1);
%                        l_link = n_link+1;
%                        l_terminal = narrow_endPts(2,:);
%                     end
                    [CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(mpData.linklength);
                    mpData.init_angle = atan2(dvec(2), dvec(1));
                    mpData.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
                    mpData.CollisionVarCritRadiusR = CollisionVarCritRadiusR;
                    % @enable_bound, do we also smaple the boundary variety
                    mpData.enable_bound = false;
                    mpData.left_2_right = true;
                    mpData.nonbsamples = 1;
                    [out] = RandomLoopGeneratorGen2(mpData);
                    % if any of the subchain has no IK solution, return
                    if size(out, 2) == 0
                        topo_state = [];  % clear topo_state if any subchain sampling fails
                        topo_state_c = [];
                        return;
                    end
                    topo_state = [topo_state; [out(1:end-1,1); angle]];
                 end
                 %remaining piece of chain, which is purely open, just randomly
                 %sample
                 if base_link <= this.num_links
                    if this.tree.mode == "open" 
                       remain_chain = (this.tree.max_ang(base_link:this.num_links) -...
                                     this.tree.min_ang(base_link:this.num_links)) .* rand(this.num_links-base_link+1,1)...
                                     + this.tree.min_ang(base_link:this.num_links);
                       topo_state = [topo_state; remain_chain];
                    else
                       %% now create open or closed chains to apply IK for sampling their configurations.
                       dvec = this.holds.coord(:, this.tree.hold_id_r) - base_terminal;
                       mpData.linklength = [this.tree.linkLengthVec(base_link:this.num_links);norm(dvec)];
                       [CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(mpData.linklength);
                       mpData.initAngle = atan2(dvec(2), dvec(1));
                       mpData.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
                       mpData.CollisionVarCritRadiusR = CollisionVarCritRadiusR;
                       % @enable_bound, do we also smaple the boundary variety
                       mpData.enable_bound = false;
                       mpData.left_2_right = true;
                       mpData.nonbsamples = 1;
                       [remain_chain] = RandomLoopGeneratorGen2(mpData);
                       % if any of the subchain has no IK solution, return
                       % and clear topo_state
                       if size(remain_chain,2)==0
                          topo_state = [];
                          topo_state_c = [];
                          return;
                       end 
                       % sz_topo = size(topo_state, 2);
                       remain = remain_chain(:,1);
%                        sz_remain = size(remain, 2);
%                        if sz_topo~=sz_remain
%                            fprintf("in sampleTopo(), size of remain and size of topo is not matching \n");
%                            topo_state = [];
%                            return;
%                        else
                        topo_state = [topo_state; remain(1:end-1)];
                       % end
                    end
                 end
                 collision = this.CheckCollision(topo_state);
                 numCollisionChecks = numCollisionChecks + 1;
                 if ~collision
                   link_pos = this.generate_link_position(topo_state);
                   topo_state_c = [topo_state_c, link_pos];
                   % this.draw_config_and_map(link_pos,1,'r');
                 else
                   topo_state = [];
                   topo_state_c = [];
                 end
              end
           end
        end
       

        function [node, node_index] = nearest(this, new_node)
            %%% 1: nearest by joint positions
            node_index = this.next_node_by_len(new_node);
            node = this.tree.angle(:, node_index);
            return;
            %%% 2: nearest by angle distance
            %node_index = this.next_node_by_ang(new_node);
        end

        function [ nearest_node ] = next_node_by_ang(this, temp_ang)
            [~, minFromPointInd] = min(vecnorm(this.tree.angle -temp_ang));
            nearest_node = minFromPointInd;
        end
        %% find nearest neighbor based upon 
        function [ nearest_node ] = next_node_by_len(this, temp_ang)
            link_pos_temp = this.generate_link_position(temp_ang);
            %tip = [squeeze(this.tree.position(end, 1, 1:this.nodes_added-1))';
            %       squeeze(this.tree.position(end, 2, 1:this.nodes_added-1))'];
            %distsq_to_goal = vecnorm(tip - this.holds.coord(:, this.tree.hold_id_r)).^2;
            % find index of minimumal distance node 
            [~, minFromPointInd] = min(...
                sum((squeeze(this.tree.position(:, 1, 1:this.nodes_added-1)) - repmat(link_pos_temp(:,1), 1, this.nodes_added-1)) .^ 2) ...
                + sum((squeeze(this.tree.position(:, 2, 1:this.nodes_added-1)) - repmat(link_pos_temp(:,2), 1, this.nodes_added-1)) .^ 2));
                %+ 0.5 * distsq_to_goal);
            nearest_node = minFromPointInd;
        end
        
        %% rrt steering method: given a new node and its nearest node id, compute
        % the fastest steering position toward this new node
        function position = steer(this, from, new_node)  %%%%% need to modify based upon chain is open or closed
            % in this function we try to go in the direction of a given
            % new_node node if it is to far and we cannot reach it in one
            % step. 
            
            % from = this.tree.angle(:, nearest_node_ind);
            to = new_node;
            angle_diff = ConvertNormal(to - from);
            node = zeros(this.num_links, 1);
            angle_inds = abs(angle_diff) > this.delta_ang_max;
            
            % Making the state nice if it was far away from the nearest state
            
            node(angle_inds) = from(angle_inds) + sign(angle_diff(angle_inds)) * this.delta_ang_max;
            node(~angle_inds) = to(~angle_inds);
            
            % return the generated state
            if strcmp(this.tree.mode, "open")
              position = node;
            else
              %% closing the loop
              %compute the left end, and right end of the remaining 2-link
              %closed chain
              xsum = sum(this.tree.linkLengthVec(1:this.num_links-2).* cos(node(1:this.num_links-2)));
              ysum = sum(this.tree.linkLengthVec(1:this.num_links-2).* sin(node(1:this.num_links-2)));
              left_end = this.tree.base + [xsum;ysum];
              right_end = this.holds.coord(:, this.tree.hold_id_r);
              [interv] = cal_ik_2pt(this.tree.linkLengthVec(this.num_links),...
                     this.tree.linkLengthVec(this.num_links-1), left_end, right_end,2,1);
               if size(interv,2) > 0
                  position = [node(1:this.num_links-2); interv(:,1)];
               else
                  % close the loop using pseudo-inverse
                  dv = right_end - left_end;
                  norm_dv = norm(dv);
                  lastAngle = atan2(dv(2), dv(1));
                  if norm_dv > sum(this.tree.linkLengthVec(this.num_links-1: this.num_links))
                      position = [node(1:this.num_links-2); lastAngle; lastAngle];
                  else
                     if norm_dv < this.tree.linkLengthVec(this.num_links-1) - this.tree.linkLengthVec(this.num_links)
                         position = [node(1:this.num_links-2); lastAngle; lastAngle + pi];
                     else
                         position = [node(1:this.num_links-2); lastAngle + pi; lastAngle];
                     end
                  end
                  [j1,c1] = this.closeLoopByPseudoIK(position);
                  if size(j1,2) > 0
                     position = j1(:,end);
                  else
                     position = [];
                  end
               end
            end
        end
        
        %% rrt connect method: given a new node and its nearest node id, see if
        %% the tree can connect to this new node directly
        %@param: nearest_node_ind,  index of the nearest node
        %@param: new_node,  joint angles of newly sampled configuration
        %outputs:
        %@param new_sample, the final succeesful sample to be added to the
        %tree, note here we only added the last successful smaple into tree
        %@param connect_status? "Reached", means the tree is extended all way to
        %new_node, "Advanced" means the tree stops in the middle of
        %extending, but at least advanced a little, "Trapped" means no
        %advancing at all
        function [new_sample, connect_status, numCollisionChecks] = connect(this, nearest_node, new_node)
            % in this function we try to go in the direction of a given
            % new_node node from the nearest_node. The status could be
            % advancing (at least advancing one step), trapped (can not move at all),
            % or reached
            numCollisionChecks = 0;
            advanced = false;
            from  = nearest_node;
            while true, 
              new_sample = this.steer(from, new_node);
              collision = this.obstacle_collision(new_sample, from);
              numCollisionChecks = numCollisionChecks + 1;
              if ~collision
                  distance = this.joint_metric(new_sample, new_node);
                  advanced = true; % at least advanced
                  if distance < this.conf.delta_ang_max
                      connect_status = "Reached";
                      return;
                  else
                      connect_status = "Advanced";
                      from = new_sample;
                  end
              else
                 if ~advanced 
                    connect_status = "Trapped";
                 else
                     connect_status = "Advanced";
                 end
                 new_sample = from;
                 return;
              end
            end
        end

        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            if strcmp(map_name ,'null.mat')
                %% holds is a structure
                this.holds.coord = cell(1,1);
                this.holds.num = 0;
                this.XY_BOUNDARY = [-45 45 0 90];
                this.obstacle.numObst = 0;
                this.obstacle.center = [];
                this.obstacle.coord = [];
            else
                map_path = 'maps/';
                this.holds = load([map_path map_name], 'num', 'coord', 'x_constraints', 'y_constraints');
                % this.obstacle.numObst --- number of obstacle
                % this.obstacle.obstDesc{i}.vertex --- 2 by k matrix, each column
                % is a vertex
                this.obstacle = load([map_path map_name],'numObst', 'obstDesc');
                this.obstacle.center = cell(this.obstacle.numObst, 1);
                for i=1:this.obstacle.numObst,
                    numVert = size(this.obstacle.obstDesc{i}, 2);
                    this.obstacle.center{i} = this.obstacle.obstDesc{i} * ones(numVert, 1) * 1.0 / numVert;
                end
                this.obstacle.fv = [];
                % preprocess obstacle (triangulation for collission
                % checking later one
                this.processObstacle();
                this.XY_BOUNDARY = [this.holds.x_constraints this.holds.y_constraints];
            end
        end
        
        function distance = joint_metric(this, jnt1, jnt2)
            delta = ConvertNormal(jnt2 - jnt1);
            distance = norm(delta, inf);
            return;
        end

        function collision = obstacle_collision(this, new_node, old_node)
            
            % generate this.step_div number of states to prevent omission
            delta = ConvertNormal(new_node - old_node);    % this.tree.angle(:, node_index));
            dist = norm(delta); %sum(abs(deltanew));
            if dist < this.conf.epsilon
                collision = false;
                return;
            end
            deltanew = delta/dist;  % normalize deltanew;
            dist_vec = 0:this.conf.step_delta:dist;
            this.turn_states = old_node + deltanew * dist_vec;

%             for link_ind = 1:this.num_links
%                 % for joint, might be interpolate the difference
%                 angDiff = ConvertNormal(new_node(link_ind) - this.tree.angle(link_ind, node_index));
%                 this.turn_states(link_ind, :) = this.tree.angle(link_ind, node_index) + linspace(0, angDiff, this.step_div);
%             end
            
            % generate positions for all the states we created
            for ind1 = 1:size(this.turn_states, 2)
                  collision = this.CheckCollision(this.turn_states(:, ind1));
                  if collision
                      return
                  end
            end
        end
        
        %% collision checking functions
        function collision = CheckCollision(this, angle)
            % first: link-obstacle collision check 
            [fv, fvList] = triangleChain(this.tree.base',this.tree.linkLengthVec, this.tree.linkWidthVec, angle);
            collision = CollisionCheck(fv, this.obstacle.fv);
            if collision
                return;
            end
            % second: link-link collision check (self collision)
            
            for ind3 = 1:this.num_comb
                ind_links = this.test_list(ind3,:);
                collision = CollisionCheck(fvList{ind_links(1)}, fvList{ind_links(2)});
                if collision
                   return;
                end
            end
        end
      
        % @isNarrowAndTopo
        function [new_node_ind, succ] = insert_node(this, parent_node_ind, new_node, isNarrowAndTopo)
            % method insert new node in the tree
            succ = true;
            new_node_ind = this.nodes_added;
            if new_node_ind > this.max_nodes
                succ = false;
            end
            this.nodes_added = this.nodes_added + 1;
            this.tree.angle(:, new_node_ind) = new_node;
            this.update_link_position(new_node_ind);
            this.parent(new_node_ind) = parent_node_ind;
            this.tree.narrow_topo(new_node_ind) = isNarrowAndTopo;
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1; 
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node, nearest_node_ind)
            % why do we seek nearest using link positions, however
            % neighbors using angles?
            % clarify this
            temp = abs(this.tree.angle(:, 1:this.nodes_added-1) - repmat(new_node, 1, this.nodes_added-1)) > this.delta_ang_neighbor;
            %temp(:, nearest_node_ind) = this.tree.angle(:, 1) * 0;
            ind = 1:this.nodes_added-1;
            temp = ind(~sum(temp));
            neighbor_nodes = setdiff(temp, nearest_node_ind);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %% final_path_j and final_path_c should already be smoothed out
        function plot(this, final_path_j, final_path_c, final_terminal)
            radius = 2;
            % obtain path
            %%[backtrace_path, path_iter] = this.evaluate_path();
            % smoothen the trajectory. manipulator will move without jerks
            % the following function should be called during each RRT path
            % calculation
            %%[index_array, smooth_ang, smooth_trajectory ] = this.smooth_trajectory(final_path_j);
            %[ xx, yy, trajectory] = this.smooth_trajectory(final_path_j, final_path_c);
            fig1 = figure;
            % make plot window bigger
            set(fig1, 'Position', [50 50 1600 800]);
            
            % two subplot is on the plot
            % left subplot will draw the manipulator's motion
            % right one will draw the tree of the position of end effector
            
            subplot(1,2,1);
            % preparing to draw circle which will indicate the destination
            hold on;
            p1 = this.plot_circle(this.goal_point(1), this.goal_point(2), this.delta_goal_point);
            set(p1, 'HandleVisibility','off');
            
            % holds and obstacle drawing
            this.draw_holds();
            this.draw_obstacles();
            
            grid on; axis square; axis(this.XY_BOUNDARY);
            title('Climbing robot','FontWeight','Demi');
            xlabel(' Position (m) ' , 'FontWeight', 'Demi');
            ylabel(' Position (m) ' , 'FontWeight', 'Demi');
            hold off;
            
            %% RRT tree will not be drawn, due to that we can draw it for one gait, but not for all gaits
            % right plot
            subplot(1,2,2);
            hold on;
            
%             % plots the tree of end effector position
%             drawn_nodes = zeros(1, this.nodes_added-1);
%             for ind = this.nodes_added-1:-1:1;
%                 current_index = ind;
%                 while(current_index ~= 1) && this.parent(current_index) ~= -1
%                     % avoid drawing same nodes twice or more times
%                     if(drawn_nodes(current_index) == false || drawn_nodes(this.parent(current_index)) == false)
%                         plot([this.tree.position(end, 1, current_index);this.tree.position(end, 1, this.parent(current_index))], ...
%                             [this.tree.position(end, 2, current_index);this.tree.position(end, 2, this.parent(current_index))],'g-','LineWidth', 0.1);
%                         plot([this.tree.position(end, 1, current_index);this.tree.position(end, 1, this.parent(current_index))], ...
%                             [this.tree.position(end, 2, current_index);this.tree.position(end, 2, this.parent(current_index))],'.k');
%                         drawn_nodes(current_index) = true;
%                     end
%                     current_index = this.parent(current_index);
%                 end
%             end
%             
            p1 = plot(squeeze(final_path_c(:, 1, :))', squeeze(final_path_c(:,2,:))' , 'LineWidth', 3);
            %
            
            % obstacle drawing
            this.draw_holds();
            this.draw_obstacles();
            
            grid on; axis square; axis(this.XY_BOUNDARY);
            title('Path and Rapidly Exploring Random Tree','FontWeight','Demi');
            xlabel(' Position (m) ' , 'FontWeight', 'Demi');
            ylabel(' Position (m) ' , 'FontWeight', 'Demi');
            hold off;
            
            
            %% remaining code draws the motion of the robot
            % line to fix the video output problem
            set(gcf,'renderer','opengl');
            
            % smooth_ang = zeros(this.num_link, size(xx, 2));
            % smooth_trajectory = zeros(this.num_link, 2, size(xx, 2));
            for ind = 1:size(fina_path_c,2)
                %smooth_ang(:,ind) = cumsum( yy(:,ind) );
                %smooth_trajectory(:, 1, ind) = cumsum( this.len_link.*cos(smooth_ang(:,ind) )) + this.start_point(1);
                %smooth_trajectory(:, 2, ind) = cumsum( this.len_link.*sin(smooth_ang(:,ind) )) + this.start_point(2);
                if ~final_path_t(ind)    % left teriminal
                    linkLengthVec = this.conf.len_link;
                else
                    linkLengthVec = this.conf.rev_len_link;
                end
                
                % plot on the left subplot
                subplot(1,2,1);
                cla;
                hold on;
                
                % initial joint of the redundant robot manipulator
                this.plot_circle(this.start_point(1), this.start_point(2), radius);
                for ind2=1:this.num_link
                    
                    fill([ final_path_c(ind2, 1, ind) - radius * sin(final_path_j(ind2,ind)); ...
                        final_path_c(ind2, 1, ind) - radius * sin(final_path_j(ind2,ind)) - linkLengthVec(ind2) * cos(final_path_j(ind2,ind));       ...
                        final_path_c(ind2, 1, ind) + radius * sin(final_path_j(ind2,ind)) - linkLengthVec(ind2) * cos(final_path_j(ind2,ind)); ...
                        final_path_c(ind2, 1, ind) + radius * sin(final_path_j(ind2,ind))], ...
                        [ final_path_c(ind2, 2, ind) + radius * cos(final_path_j(ind2,ind)); ...
                        final_path_c(ind2, 2, ind) + radius * cos(final_path_j(ind2,ind)) - linkLengthVec(ind2) * sin(final_path_j(ind2,ind)) ; ...
                        final_path_c(ind2, 2, ind) - radius * cos(final_path_j(ind2,ind)) - linkLengthVec(ind2) * sin(final_path_j(ind2,ind)); ...
                        final_path_c(ind2, 2, ind) - radius * cos(final_path_j(ind2,ind))] ...
                        ,'g');
                    
                    s3 = this.plot_circle(final_path_c(ind2, 1, ind), final_path_c(ind2, 2, ind), radius);
                end
                drawnow; % draw the manipulator immediately 
                hold off;
            end
        end
        
        function newObj = copyobj(thisObj)
            % Construct a new object based on a deep copy of the current
            % object of this class by copying properties over.
            props = properties(thisObj);
            for i = 1:length(props)
                % Use Dynamic Expressions to copy the required property.
                % For more info on usage of Dynamic Expressions, refer to
                % the section "Creating Field Names Dynamically" in:
                % web([docroot '/techdoc/matlab_prog/br04bw6-38.html#br1v5a9-1'])
                newObj.(props{i}) = thisObj.(props{i});
            end
        end
        
        function update_link_position(this, ind)
            % Update info about the manipulator
            world_ang = this.tree.angle(:, ind);   %cumsum( this.tree.angle(:, ind) );
            % Compute the link end positions
            this.tree.position(:, 1, ind) = [this.tree.base(1); cumsum( this.tree.linkLengthVec.*cos(world_ang )) + this.tree.base(1)];
            this.tree.position(:, 2, ind) = [this.tree.base(2); cumsum( this.tree.linkLengthVec.*sin(world_ang )) + this.tree.base(2)];
        end
        
%         function robot_c = compute_link_position(this, world_ang)
%             % Update info about the manipulator
%             % world_ang = this.tree.angle(:, ind);   %cumsum( this.tree.angle(:, ind) );
%             % Compute the link end positions
%             robot_c(:, 1) = cumsum( this.tree.linkLengthVec.*cos(world_ang )) + this.tree.base(1);
%             robot_c(:, 2) = cumsum( this.tree.linkLengthVec.*sin(world_ang )) + this.tree.base(2);
%         end
        
        function link_pos = generate_link_position(this, temp_world_ang)
            % generates position and orientation of the redundant
            % manipulator model
            link_pos = zeros(this.num_links+1, 2);
            % temp_world_ang = cumsum( node );
            % Compute the link end positions
            link_pos(:, 1) = [this.tree.base(1); cumsum( this.tree.linkLengthVec.*cos(temp_world_ang)) + this.tree.base(1)];
            link_pos(:, 2) = [this.tree.base(2); cumsum( this.tree.linkLengthVec.*sin(temp_world_ang)) + this.tree.base(2)];
        end
        
%         
%         function cost = calc_cost_euclidian(this, from_ind, dest_node)
%             % square root is not taken
%             cost = norm(this.tree.position(:, :, from_ind) - dest_node, 'fro');
%         end
%         
%         function cost = calc_cost_angle(this, from_ind, dest_node)
%             % square root is not taken
%             cost = norm(this.tree.angle(:, from_ind) - dest_node, 'fro');
%         end
        %@path_j, raw joint-space path
        %@path_c, raw cartesian-space path (Cartesian coordinates of all link joints)
        %@narrow_topo, vector of 0/1s indicating if a sample is narrow and
        %coming from topological component
        %@smooth_j, joint-space path after interpolation
        %@smooth_c, Cartsian paths after interpolation
        function [path_j, path_c, narrow_topo, smooth_j, smooth_c, path_narrowness_raw, path_clearance_raw] = retrieve_path(this, basedEndPts, goal_index)
            path_j=[]; path_c=[]; path_narrowness_raw=[]; path_clearance_raw = []; narrow_topo=[];
            smooth_j = []; smooth_c =[];
            [backtrace_path, path_iter] = this.evaluate_path(basedEndPts, goal_index);
            if path_iter > 0
              path_j = this.tree.angle(:, backtrace_path(end:-1:1));
              narrow_topo = this.tree.narrow_topo(:, backtrace_path(end:-1:1));
              if this.joint_metric(path_j(:,end), this.tree.goal_angle) > this.conf.angle_same_epsilon && ~basedEndPts
                  path_j = [path_j, this.tree.goal_angle];
                  narrow_topo = [narrow_topo, 0];
              end
              for i=1:size(path_j,2),
                 [narrowness] = this.calNarrowNess(path_j(:,i)); % this.calMinClearance(path_j(:,i));
                 path_narrowness_raw = [path_narrowness_raw, narrowness];
                 [clearance] = this.calMinClearance(path_j(:,i));
                 path_clearance_raw = [path_clearance_raw, clearance];
                 link_pos = this.generate_link_position(path_j(:,i));
                 path_c = [path_c, link_pos]; 
              end
              
              [smooth_j, smooth_c] = this.linear_smooth_trajectory(path_j); %this.smooth_trajectory_function(path_j);
              %path_j = smooth_ang;
              %path_c = smooth_trajectory;
              if this.conf.animate_raw 
                 if this.conf.animate_every_step
                    this.Animation_path_j(path_j, path_c, narrow_topo, this.conf.animation_filename);
                 end
              else
                 if this.conf.animate_every_step 
                   this.Animation_path_j(smooth_j, smooth_c, zeros(1, size(path_j,2)), this.conf.animation_filename); 
                 end
              end
           end
        end
        % close the loop of an open chain by psedo inverse method, 
        % but need to do collision checking
        function [j1,c1] = closeLoopByPseudoIK(this, initAngle)
             j1=[]; c1 = [];
             linklength = this.tree.linkLengthVec;
             startPt = this.tree.base;
             endPt = this.holds.coord(:, this.tree.hold_id_r);
             
             if length(linklength) < 2
                fprintf("the DOF of the chain is less than 2, can not close loop using pseudo inverse algorithm");
                return;
             end
             angle = initAngle;
             newStartPt = startPt + [sum(linklength' .* cos(angle')); sum(linklength' .* sin(angle'))];
             dv = endPt - newStartPt;
             % jacobian matrix
             J = [-linklength'.* sin(angle'); linklength'.* cos(angle')];
             numSteps = 0;
             while norm(dv) > this.conf.dvEpsilon && numSteps < this.conf.maxLoopCloseSteps
                dtheta = J' * inv(J * J') * dv;
                angle = angle + this.conf.accordion_coef * dtheta;
                %collision = this.CheckCollision(angle);
                %if ~collision
                   %j1 = [j1, angle];
                   J = [-linklength'.* sin(angle'); linklength'.* cos(angle')];
                   newStartPt = startPt + [sum(linklength' .* cos(angle')); sum(linklength' .* sin(angle'))];
                   dv = endPt - newStartPt;
                   numSteps = numSteps + 1;
                %else
                %   fprintf("close loop by pseudo inverse generates position in collision during iteration");
                %   j1 = [];
                %   return;
                %   break;
                %end
             end
             if numSteps >= this.conf.maxLoopCloseSteps
                fprintf("close loop by pseudo inverse does not converge");
                j1 = [];  %% return empty
                return;
             end
             %num_conf = size(j1,2);
             %for i=1:num_conf
             j1 = angle;
             c1 = this.generate_link_position(j1);
             %%this.draw_config_and_map(c1,3,'c');
             %end
        end
       
        %% for open chains, because there is no exact state for a chain based at a hold that tries to reach another hold,
        % we evaluate path based upon end point, for closed chains, we have
        % some fixed state for some angles
        % @baseEndPts = true if the path is for an open chain to reach a
        % new hold? = false if the path is for a closed chain with two
        % fixed terminals.
        function [backtrace_path, path_iter, reach_goal] = evaluate_path(this, basedEndPts, goal_index)
            % method find the cheapest path and the number if nodes in this
            % path
            backtrace_path = [];
            distances = zeros(this.nodes_added-1, 2);
            if goal_index < 0
              if basedEndPts   % if based upon tip point matching, then compute the distances of samples to the goal hold based upon tip distance
                 goal_holdPt = this.holds.coord(:, this.tree.hold_id_r);  
                 distances(:, 1) = (this.tree.position(end, 1, 1:this.nodes_added-1) - goal_holdPt(1)) .^ 2 ...
                   + (this.tree.position(end, 2, 1:this.nodes_added-1) -  goal_holdPt(2)) .^ 2;
              else  % otherwise, we know the goal config, so we compute the joint distance
                 distances(:, 1) = vecnorm(this.tree.angle(:, 1:this.nodes_added-1) - this.tree.goal_angle, inf);
              end
              distances(:, 2) = 1:this.nodes_added-1;
              distances = sortrows(distances, 1);
              fprintf("smallest distance to goal is %f \n", distances(1,1));
              if basedEndPts  %% if based on tip point matching, compare it with delta_goal_point
                 dist_index = find(distances(:, 1) <= (this.conf.delta_goal_point ^ 2));
              else
                 dist_index = find(distances(:, 1) <= this.conf.delta_goal_angle); 
              end
              num_close_indices = length(dist_index);
              if num_close_indices == 0
                  reach_goal = false;
              else
                  reach_goal = true;
                
              end
              if ~basedEndPts  % for closed chain, sometimes it is very hard to exactly reach goal joint vector, we need to verify path existence for all existing leafs
                  num_close_indices = this.nodes_added - 1;
              else
                  num_close_indices = 1;  %% aleays use the closest point  
              end
            
              %% here needs to check if there is a path between 
              nearest_node_index = -1;
              %min_cost = inf;
              for i=1:num_close_indices,
                 %ind = dist_index(i);
                 node_index = distances(i,2);
                 %% check if there is a simple collision free path from  angle_index to the goal point
                 collision = false;
                 if ~basedEndPts   %% if not based on tip point matching, then goal angle is known, so have to make sure node-index to goal_angle is collision free
                   collision=this.obstacle_collision(this.tree.goal_angle, this.tree.angle(:, node_index));
                 end
                 if ~collision
%                    temp_cost = this.cumcost(distances(ind,2));
%                    if(min_cost > temp_cost)
%                       min_cost = temp_cost;
                      nearest_node_index = node_index; %distances(ind,2);
                      break;
%                     end
                 end
              end
              if nearest_node_index < 0
                disp('NOTICE! Robot cannot reach the goal');
                path_iter = 0;
                return;
              end
              current_index = nearest_node_index;
            else
              reach_goal = true;
              current_index = goal_index;
            end
              % backtracing the path
              
            path_iter = 1;
            backtrace_path = zeros(1,1);
            while(current_index ~= 1 )
               backtrace_path(path_iter) = current_index;
               path_iter = path_iter + 1;
               current_index = this.parent(current_index);
               if path_iter > 1000
                    disp('evaluate path fails');
                    break;
               end
            end
            backtrace_path(path_iter) = current_index;
        end
        
        %% plots the path and put into several figures, each show a portion of the path
        % length_c is a vector of integers, which records the accumulated number of samples in each segment
        function make_print(this, trajectory_c, narrow_topo, length_c)
           num_path_segs = length(length_c);
           previous_num_samples = 0;
           linkage_color = [.4 0 .4];
           linkage_color_narrow = "c"; % [187,86,149]/255;
           for i=1:num_path_segs,
             num_samples = length_c(i) - previous_num_samples; 
             gap = ceil(num_samples/this.conf.num_samples_in_one_plot);
             ind = previous_num_samples+1:gap:length_c(i);
             fig_id = this.conf.plot_sample_start_fig_id+i;
             figure(fig_id);
             hold on;
             xlabel('x');
             ylabel('y');
               
             % draw map
             this.draw_obstacles(fig_id);
             this.draw_holds(fig_id); 
    
             % draw start
             x_start = trajectory_c(:,1);
             y_start = trajectory_c(:,2);
             line('XData',x_start, 'YData',y_start, 'Color',[0 1 0], 'LineWidth', this.conf.max_linkwidth);
               

             % draw goal
             x_goal = trajectory_c(:, end-1);
             y_goal = trajectory_c(:, end);
             line('XData',x_goal, 'YData',y_goal, 'Color',[1 0 0], 'LineWidth', this.conf.max_linkwidth);
            
             axis equal
             for j=1:length(ind),
                 if narrow_topo(ind(j)) > 0
                    color = linkage_color_narrow;
                 else
                    color = linkage_color;
                 end
                 xpos = trajectory_c(:,2*ind(j)-1);
                 ypos = trajectory_c(:,2*ind(j));
                 line('XData',xpos, 'YData',ypos, 'Color', color, 'LineWidth', this.conf.max_linkwidth);
             end
             hold off;
             previous_num_samples = length_c(i);
          end
        end
        % linear interpolation between a sequence of sampled milestones
        function [smooth_ang, smooth_trajectory] = linear_smooth_trajectory(this, joint_ang_states)
          smooth_trajectory = [];
          smooth_ang = [];
          num_joint_states = size(joint_ang_states, 2);
          for i=1:num_joint_states-1
            [traj_out_j] = LinearLocalPlanner(joint_ang_states(:,i), joint_ang_states(:,i+1), this.conf.step_delta);
            smooth_ang = [ smooth_ang, traj_out_j];
          end
          if size(smooth_ang, 2)==0
              smooth_ang = joint_ang_states;
          end
          for ind = 1:size(smooth_ang,2)
             if  this.tree.mode ~= "open"
                 [j1] = this.closeLoopByPseudoIK(smooth_ang(:,ind));
                 if size(j1,2) > 0
                    smooth_ang(:,ind) = j1(:,end);
                 end
              end 
              % smooth_ang(:,ind) = angle;
              link_pos = this.generate_link_position(smooth_ang(:,ind));
              smooth_trajectory = [smooth_trajectory, link_pos];
          end
        end
        %% interpolation in the join space, and then compute long array of joint angles: smooth_ang,
        % and long array of cartesian coordinates of all joints in the
        % chain: smooth_trajectory
        function [smooth_ang, smooth_trajectory ] = smooth_trajectory_function(this, joint_ang_states)
            %SMOOTH_TRAJECTORY Smooth the initial trajectory to avoid jerked motion
            
%             joint_pos_states = zeros(this.num_link, 2, path_iter);
%             joint_ang_states = zeros(this.num_link, path_iter);
%             
%             for ind = path_iter:-1:1
%                 joint_pos_states(:,:, ind) = this.tree.position(:,:, backtrace_path(ind));
%                 joint_ang_states(:, ind) = this.tree.angle(:, backtrace_path(ind));
%             end
            smooth_trajectory = [];
            path_iter = size(joint_ang_states, 2);
            
            index_array = 1:0.1:path_iter;
            x = 1:1:path_iter;
            smooth_ang = zeros(this.num_links, size(index_array,2));

            % if this.tree.mode == "open"
           for temp_ind = 1:this.num_links
               smooth_ang(temp_ind, :) = cubic_spline_refine(x, squeeze(joint_ang_states(temp_ind, :)), index_array);
           end
            %  lenVec = this.tree.linkLengthVec;
            %else
            %  for temp_ind = 1:this.num_links-2
            %     smooth_ang(temp_ind, :) = cubic_spline(x, squeeze(joint_ang_states(temp_ind, :)), index_array);
            %  end 
            %  lenVec = this.tree.linkLengthVec(1:this.num_link-2);
            %end
            
            
            %smooth_trajectory = zeros(this.num_links, 2, size(index_array,2));
            %last_two_link = this.tree.linkLengthVec(this.num_links-1,this.num_links);
            for ind = 1:size(index_array,2)
                %%smooth_ang(:,ind) = yy(:, ind); %cumsum( yy(:,ind) );
                %smooth_trajectory(:, 1, ind) =this.tree.base(1) + cumsum( lenVec.*cos(smooth_ang(:,ind) ));
                %smooth_trajectory(:, 2, ind) = this.tree.base(2) + cumsum( lenVec.*sin(smooth_ang(:,ind) ));
                angle = smooth_ang(:,ind);
                % compute the sign of last angle
                %%angle_diff = ConvertNormal(angle(end)-angle(end-1));
                
                %two terminal cooridates
                if  this.tree.mode ~= "open"
%                   pl = link_pos(end-2,:)';
%                   pr = this.holds.coord(:, this.tree.hold_id_r);
%                   [interv] = cal_ik_2pt(this.tree.linkLengthVec(this.num_links),...
%                      this.tree.linkLengthVec(this.num_links-1), pl, pr,2,1);
%                   if size(interv,2) > 0
%                       if angle_diff > 0
%                          angle(this.num_links-1:this.num_links) = interv(:,1);
%                       else
%                          angle(this.num_links-1:this.num_links) = interv(:,2); 
%                       end
%                   else
                     % close the loop using pseudo-inverse
                    %close_angle = angle;
% %                      dv = pr - pl;
% %                      norm_dv = norm(dv);
% %                      lastAngle = atan2(dv(2), dv(1));
% %                      if norm_dv > sum(this.tree.linkLengthVec(this.num_links-1: this.num_links))
% %                         close_angle(this.num_links-1:this.num_links) = [lastAngle; lastAngle];
% %                      else
% %                        if norm_dv < this.tree.linkLengthVec(this.num_links-1) - this.tree.linkLengthVec(this.num_links)
% %                           close_angle(this.num_links-1:this.num_links) = [lastAngle; lastAngle + pi];
% %                        else
% %                           close_angle(this.num_links-1:this.num_links) = [lastAngle + pi; lastAngle];
% %                        end
% %                      end
                     [j1] = this.closeLoopByPseudoIK(angle);
                     if size(j1,2) > 0
                        angle = j1(:,end);
                        smooth_ang(:,ind) = angle;
                     end
                end 
                % smooth_ang(:,ind) = angle;
                link_pos = this.generate_link_position(angle);
                smooth_trajectory = [smooth_trajectory, link_pos];
            end
        end
        
        %% displaying utility functions
        function draw_holds(this, fig_id)
            figure(fig_id);
            hold on;
            for i=1:this.holds.num,
              % draw holds as black color
              this.plot_circle(this.holds.coord(1, i), this.holds.coord(2, i), this.conf.hold_radius, 'k');
            end
            hold off;
        end
        function draw_obstacles(this, fig_id)
            figure(fig_id);
            hold on;
            % method draws all obstacles
            for i=1:this.obstacle.numObst,
               coord = this.obstacle.obstDesc{i};
               sz_vert = size(coord,2);
               if sz_vert==1,
                  plot(coord(1),coord(2));
                  hold on;
               else
                   plot(polyshape(coord(1,:),coord(2,:)));
               end
            end
            hold off;
        end
        function draw_config(this, link_pos, fig_id, color)
            figure(fig_id);
            hold on;
            % draw start
            line('XData',link_pos(:,1), 'YData',link_pos(:,2), 'Color',color, 'LineWidth', this.conf.max_linkwidth);
%             [fv, fvList] = triangleChain(this.tree.base',this.tree.linkLengthVec, this.tree.linkWidthVec, joint);
%              p = patch (fv);
%              if nargin >= 4
%                 p.FaceColor = color;
%                 p.EdgeColor = color;  
%              else
%                 p.FaceColor = 'green';
%                 p.EdgeColor = 'none';
%              end
             hold off;
        end
        %% draw configuration and the map
        % @linkPos is the position of link joints
        function draw_config_and_map(this, linkPos, fig_id, robot_color)
            hnd = figure(fig_id);
            clf(hnd);
         % remove ex
            this.draw_obstacles(fig_id);
            this.draw_holds(fig_id);  
            this.draw_config(linkPos, fig_id, robot_color);
        end
        
        %% plot path clearance
        function plot_path_clearance(this, narrow_topo, final_path_clearance_raw, name)
            if strcmp(name,"narrowness")
               figure(this.conf.narrowness_fig_id);
            else
               figure(this.conf.clearance_fig_id);
            end
            num_samples = size(final_path_clearance_raw, 2);
            xaxis = 1:1:num_samples;
            plot(xaxis,final_path_clearance_raw,'b');
            hold on;
            if sum(narrow_topo) > 0
              ind = find(narrow_topo > 0);
              plot(xaxis(ind), final_path_clearance_raw(ind), 'ro');
              legend(name, 'narrow samples from topological component');
            end
            xlabel('sample indices');
            ylabel(name);
            tit = strcat('path ', name, ' from obstacles');
            title(tit);
            hold off;
        end
        
        
        %% animate the obtained path
        %animation_png_headname is the head file of all generated png image
        %sequences (often givesn as headname1000.png, headname10001.png, etc)
        function Animation_path(this, final_path_c, narrow_topo, animation_png_headname) 
           fig_hnd = figure(this.conf.animation_fig_id);
           clf(fig_hnd);
           set(fig_hnd,'Tag','png', 'Units','normalized', 'Position',[0.01 .51 .5 .5],...
               'Color',[.95 .99 1]);

          axes('XLim', [-10  10], 'Ylim', [-5  15]);
          hold on;
          
          % draw map
          this.draw_obstacles(this.conf.animation_fig_id);
          this.draw_holds(this.conf.animation_fig_id); 
          set(gca,'DataAspectRatio',[1 1 1], 'Visible','off');
          linkage_color = [.4 0 .4];
          linkage_color_narrow = "c"; %[187,86,149]/255;
  
          %linkage_color1 = [1 0 1];
          
          % draw start
          x_start = final_path_c(:,1);
          y_start = final_path_c(:,2);
          line('XData',x_start, 'YData',y_start, 'Color',[0 1 0], 'LineWidth', this.conf.max_linkwidth);
          hold on;

          % draw goal
          num_conf = size(final_path_c,2)/2;
          
          x_goal = final_path_c(:, 2 * num_conf -1);
          y_goal = final_path_c(:, 2 * num_conf);
          line('XData',x_goal, 'YData',y_goal, 'Color',[1 0 0], 'LineWidth', this.conf.max_linkwidth);

          % start animation, requires getting the initial line handle
          line_hndl1 = line('XData',x_start, 'YData',y_start,...
           'Color',linkage_color, 'EraseMode','xor', 'LineWidth', this.conf.max_linkwidth);

          png_name = [animation_png_headname, num2str(1000) '.png'];
          print(fig_hnd, '-dpng', '-r100', png_name);

          for i = 2:num_conf,
             x_pos = final_path_c(:, 2*i-1);
             y_pos = final_path_c(:, 2*i);
             if narrow_topo(i) > 0
                 color = linkage_color_narrow;
             else
                 color = linkage_color;
             end
             set(line_hndl1, 'XData',x_pos, 'YData',y_pos,'Color', color);
         
             png_name = [animation_png_headname, num2str(999+i), '.png'];
             print(fig_hnd, '-dpng', '-r100', png_name);
          end
        end     
        % animation based upon joint angles, using polygon drawing (this
        % will make link width matching with actual link width)
        %% @final_path_j, the joints-pace path
        %% @final_path_c, the c-space path
        %% @narrow_topo, indices which indicate wheter a given config is a narrow config which is from topological component sampling
        %% @animation_png_headname is animation figure file name
        function Animation_path_j(this, final_path_j, final_path_c, narrow_topo, animation_png_headname) 
           fig_hnd = figure(this.conf.animation_fig_id);
           clf(fig_hnd);
           set(fig_hnd,'Tag','png', 'Units','normalized', 'Position',[0.01 .51 .5 .5],...
               'Color',[.95 .99 1]);

          axes('XLim', [-10  10], 'Ylim', [-5  15]);
          hold on;
          
          % draw map
          this.draw_obstacles(this.conf.animation_fig_id);
          this.draw_holds(this.conf.animation_fig_id); 
          set(gca,'DataAspectRatio',[1 1 1], 'Visible','off');
          linkage_color = [.4 0 .4];
          linkage_color_narrow = "c"; %[187,86,149]/255;
  
          %linkage_color1 = [1 0 1];
          
          % draw start
          
%           x_start = final_path_c(:,1);
%           y_start = final_path_c(:,2);
          base_coord = final_path_c(1,1:2)';
          dvec = final_path_c(2:end,1:2) - final_path_c(1:end-1,1:2);
          linkLengthVec = vecnorm(dvec')';
          [fv_start] = triangleChain(base_coord', linkLengthVec, this.conf.width_link, final_path_j(:,1));
          p = patch (fv_start);
          p.FaceColor = 'g';
          p.EdgeColor = 'g'; 
          
          %line('XData',x_start, 'YData',y_start, 'Color',[0 1 0], 'LineWidth', this.conf.max_linkwidth);
          hold on;

          % draw goal
          num_conf = size(final_path_j, 2); %size(final_path_c,2)/2;
          base_coord = final_path_c(1,2*num_conf-1:2*num_conf)';
          dvec = final_path_c(2:end,2*num_conf-1:2*num_conf) - final_path_c(1:end-1,2*num_conf-1:2*num_conf);
          linkLengthVec = vecnorm(dvec')';
          
%           x_goal = final_path_c(:, 2 * num_conf -1);
%           y_goal = final_path_c(:, 2 * num_conf);
%           line('XData',x_goal, 'YData',y_goal, 'Color',[1 0 0], 'LineWidth', this.conf.max_linkwidth);
          [fv_goal] = triangleChain(base_coord', linkLengthVec, this.conf.width_link, final_path_j(:, end));
          p = patch (fv_goal);
          p.FaceColor = 'r';
          p.EdgeColor = 'r'; 
          
          % start animation, requires getting the initial line handle
%           line_hndl1 = line('XData',x_start, 'YData',y_start,...
%            'Color',linkage_color, 'EraseMode','xor', 'LineWidth', this.conf.max_linkwidth);
          p1 = patch(fv_start, 'FaceColor', linkage_color, 'EdgeColor', linkage_color, 'EraseMode','xor');

          png_name = [animation_png_headname, num2str(1000) '.png'];
          print(fig_hnd, '-dpng', '-r100', png_name);

          for i = 2:num_conf,
             base_coord = final_path_c(1,2*i-1:2*i)';
             dvec = final_path_c(2:end,2*i-1:2*i) - final_path_c(1:end-1,2*i-1:2*i);
             linkLengthVec = vecnorm(dvec')'; 
             [fv] = triangleChain(base_coord', linkLengthVec, this.conf.width_link, final_path_j(:, i));
             if narrow_topo(i) > 0
                 color = linkage_color_narrow;
             else
                 color = linkage_color;
             end
             set(p1, 'Faces',fv.faces, 'Vertices',fv.vertices,'FaceColor', color, 'EdgeColor', color);
         
             png_name = [animation_png_headname, num2str(999+i), '.png'];
             print(fig_hnd, '-dpng', '-r100', png_name);
          end
        end        
        
        function make_print_j(this, trajectory_j, trajectory_c, narrow_topo, length_c)
           num_path_segs = length(length_c);
           previous_num_samples = 0;
           linkage_color = [.4 0 .4];
           linkage_color_narrow = "c"; % [187,86,149]/255;
           for i=1:num_path_segs,
             num_samples = length_c(i) - previous_num_samples; 
             gap = ceil(num_samples/this.conf.num_samples_in_one_plot);
             ind = previous_num_samples+1:gap:length_c(i);
             fig_id = this.conf.plot_sample_start_fig_id+i;
             figure(fig_id);
             hold on;
             xlabel('x');
             ylabel('y');
               
             % draw map
             this.draw_obstacles(fig_id);
             this.draw_holds(fig_id); 
    
             % draw start
%              x_start = trajectory_c(:,1);
%              y_start = trajectory_c(:,2);
%              line('XData',x_start, 'YData',y_start, 'Color',[0 1 0], 'LineWidth', this.conf.max_linkwidth);
             
             base_coord = trajectory_c(1,1:2)';
             dvec = trajectory_c(2:end,1:2) - trajectory_c(1:end-1,1:2);
             linkLengthVec = vecnorm(dvec')';
             [fv_start] = triangleChain(base_coord', linkLengthVec, this.conf.width_link, trajectory_j(:,1));
             p = patch (fv_start);
             p.FaceColor = 'g';
             p.EdgeColor = 'g'; 
               

             % draw goal
%              x_goal = trajectory_c(:, end-1);
%              y_goal = trajectory_c(:, end);
%              line('XData',x_goal, 'YData',y_goal, 'Color',[1 0 0], 'LineWidth', this.conf.max_linkwidth);
             
             num_conf = size(trajectory_j, 2); %size(final_path_c,2)/2;
             base_coord = trajectory_c(1,2*num_conf-1:2*num_conf)';
             dvec = trajectory_c(2:end,2*num_conf-1:2*num_conf) - trajectory_c(1:end-1,2*num_conf-1:2*num_conf);
             linkLengthVec = vecnorm(dvec')';
          

             [fv_goal] = triangleChain(base_coord', linkLengthVec, this.conf.width_link, trajectory_j(:, end));
             p = patch (fv_goal);
             p.FaceColor = 'r';
             p.EdgeColor = 'r'; 
             
            
             axis equal
             for j=1:length(ind),
                 if narrow_topo(ind(j)) > 0
                    color = linkage_color_narrow;
                 else
                    color = linkage_color;
                 end
%                  xpos = trajectory_c(:,2*ind(j)-1);
%                  ypos = trajectory_c(:,2*ind(j));
%                  line('XData',xpos, 'YData',ypos, 'Color', color, 'LineWidth', this.conf.max_linkwidth);
                 base_coord = trajectory_c(1,2*ind(j)-1:2*ind(j))';
                 dvec = trajectory_c(2:end,2*ind(j)-1:2*ind(j)) - trajectory_c(1:end-1,2*ind(j)-1:2*ind(j));
                 linkLengthVec = vecnorm(dvec')'; 
                 [fv] = triangleChain(base_coord', linkLengthVec, this.conf.width_link, trajectory_j(:, ind(j)));
                 p = patch (fv);
                 p.FaceColor = color;
                 p.EdgeColor = color; 
             end
             hold off;
             previous_num_samples = length_c(i);
          end
        end
    end
    
    methods(Static)
        function hCirc = plot_circle(x, y, r, in_color)
            if nargin == 3
              color = 'b';
            else
              color = in_color;  
            end
            draw_mode = [color, '-'];
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            hCirc = plot(cir_x, cir_y, draw_mode, 'LineWidth', 1.5);
        end
        
        function hDisk = plot_disk(x, y, r)
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            hDisk = fill(cir_x, cir_y, 'r');
        end
        
        function delay(sec)
            tic;
            while toc < sec
            end
        end
    end
end