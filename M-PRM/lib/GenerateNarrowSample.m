function [sample] = GenerateNarrowSample(mpData,compData)
%% this function always keeps sampling configs such that links falls between obstacles, 
%% it always randomly choosing an obstacle, and then 
%% check if it can be reachable
linklength = mpData.linklength;  %% linklength vector
l_len = mpData.l_len;    %% workspace boundary circles of subchain from left
r_len = mpData.r_len;
l_len_r = mpData.l_len_r;  %% workspace boundary circles of subchains from right
r_len_r = mpData.r_len_r;
numObst = compData.numObst;  %% number of obstacles
DOF = length(linklength);
sample = [];  %% initialize output sample
leftConfig = [];   %% joint angle vector for left subchain
b_sam = [];   %%  joint angle vector for left subchain that lies on B
rightConfig = []; %% joint angle vector for right subchain
start_link_index = 3;  %% start link index, always from 3, otherwise it can not be closed by left subchain
start_obst_index = -1;  %% initialize the visited obstacle index
start_vert_index = -1;  %% initialize the visited vertex index
gap_link_vert = -1;  % if gap_link_vert > 0, we still need to continue tracing toward the current vert index
max_step = 0;  %% maximal interation steps
num_left = 0; %% number of leftConfigs
leftDOF = 0;  %% DOFs of left subchain
while start_link_index < DOF - 2  && max_step < 10000
    %start_link_index = start_link_index + 1;
    %% choose the obstacle 
    find_vert = false; %% each iteration have to check if a link index has found a corresponding compatible vert index
    if start_obst_index == -1
        l_crit = l_len(start_link_index-1);
        r_crit = r_len(start_link_index-1);
        new_obst_index = ceil(rand(1) * numObst);
        vert = compData.obst(new_obst_index).vert;
        dist = compData.obst(new_obst_index).obstDist;
        angle_intv = compData.obst(new_obst_index).angle_intv;
        num_vert = size(vert,2);
        start_vert_index = ceil(rand(1) * num_vert);
        while dist(start_vert_index) > r_crit && start_link_index < DOF-2
            start_link_index = start_link_index +1;
            l_crit = l_len(start_link_index-1);
            r_crit = r_len(start_link_index-1);
        end
        if start_link_index < DOF-2
            clink = linklength(start_link_index);  %% target collision link
            pt = vert(:, start_vert_index);
            intv =  angle_intv(:,start_vert_index);
            numLeft = 0;
            numIt = 0;
            while numLeft == 0 &&  numIt < 100,
              %% first, randomly pick a ratio of clink at which pt lies  
               len_ratio = rand(1);
              %% second, randomly pick an angle of 
               which_intv = rand(1);
               ang_ratio = rand(1);
               if which_intv  >= 0.5,
                  angle = intv(1) + (intv(2)-intv(1)) * ang_ratio;
               else
                  angle = pi + intv(1) + (intv(2)-intv(1)) * ang_ratio;
               end

               l1 = clink * len_ratio;
               l2 = clink - l1;
               pt1 = pt + [l1 * cos(angle), l1 * sin(angle)]';
               pt2 = pt - [l2 * cos(angle), l2 * sin(angle)]';

              %% the entire chain brakes at point pt1, for the left part, directly
              %% call RandomLoopGeneratorPoly() function
               newlinklength = [];
               newlinklength(1:start_link_index - 1) = linklength(1:start_link_index - 1);
               newlinklength(start_link_index) = norm(pt1);
               mpData1 = mpData;
               mpData1.init_angle = atan2(pt1(2),pt1(1));
    
           
              mpData1.enable_bound = true;
              mpData1.left_2_right = false;
              mpData1.linklength = newlinklength;
              mpData1.nonbsamples = 1;
              [leftConfig,b_sam] = RandomLoopGeneratorGen2(mpData1);
              leftConfig = [leftConfig, b_sam];
              numLeft = size(leftConfig,2);
              numIt = numIt + 1;
           end
           leftDOF = start_link_index - 1;
           if numLeft > 0
              find_vert = true;
              leftConfig = leftConfig(1:leftDOF,:);
              leftConfig = [leftConfig;(angle+pi) * ones(1,numLeft)];
              leftDOF = leftDOF + 1;
              start_obst_index = new_obst_index;
           end
        end
     else
         new_obst_index = ceil(rand(1) * numObst);
         start_tracing_vertex = false;
         if new_obst_index ~= start_obst_index
            vert = compData.obst(new_obst_index).vert;
            dist = compData.obst(new_obst_index).obstDist;
            angle_intv = compData.obst(new_obst_index).angle_intv;
            num_vert = size(vert,2);
            start_vert_index = ceil(rand(1) * num_vert);
            start_obst_index = new_obst_index;
            start_tracing_vertex = true;
         else
           if gap_link_vert <= 0  
              start_tracing_vertex = true;
           else
               rightConfig = [rightConfig; ang];
               pt2 = pt2 + linklength(start_link_index) * [cos(ang),sin(ang)]';
               find_vert = true;
           end
         end
         if start_tracing_vertex
              in_interval = false;
              count = 0;
              while ~in_interval && count < num_vert
                ccw_value = rand(1);
                if ccw_value > 0.5   % ccw
                   start_vert_index = mod(start_vert_index + 1, num_vert);
                else
                   start_vert_index = mod(start_vert_index - 1, num_vert);
                end
                if start_vert_index == 0
                   start_vert_index = num_vert; %8;
                end
                diffVec = vert(:,start_vert_index) - pt2;
                ang = atan2(diffVec(2), diffVec(1));
                in_interval = CheckAngleInInterval(ang, angle_intv(:,start_vert_index));
                count = count + 1;
              end
              if in_interval
                  rightConfig = [rightConfig; ang];
                  pt2 = pt2 + linklength(start_link_index) * [cos(ang),sin(ang)]';
                  gap_link_vert = norm(diffVec) - linklength(start_link_index);
                  find_vert = true;
              end
         end
    end
    if find_vert  %% if the link start_link_index has found the target compatible vertex, incease link index
       start_link_index = start_link_index + 1;
    end
    max_step = max_step + 1;
end

rightDOF = length(rightConfig);
sample_DOF = leftDOF + rightDOF; 
if sample_DOF == DOF - 3 
   [interv] = cal_ik_2pt(linklength(DOF-2),linklength(DOF-1),pt2,[linklength(DOF),0]');  %% tomorrrow debug here
   numSol = size(interv,2);
   if numSol == 2
      choose = rand(1);
      if choose > 0.5
         rightConfig = [rightConfig; interv(:,2);pi];
      else
         rightConfig = [rightConfig; interv(:,1);pi];
      end
   else
      if numSol == 1
         rightConfig = [rightConfig; interv;pi];
      end    
   end
   if numSol > 0
       sample = [leftConfig; rightConfig * ones(1,numLeft)];
   end
end