function [newConfig]=MidLinkRandomLoopGeneratorEff(linklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,...
                                                vert, angle_intv,...
                                                pt, start_link_index, start_vert_index, peri, nobst_samples)
%% this is an important function for sampling the conformation space of a closed loop, but with link j touches point pt,
%% with orientation angles inside the interval determined by intv
%% @linklength,  the linklength vector of the closed chain

%% @CollisionVarCritRadiusL, the set of critical circles counting from the left anchor [0,0]
%% basically, CollisionVarCritRadiusL(1).linklenghth = [abs(l1-l2), l1+l2],
%%  CollisionVarCritRadiusL(2).linklength = criticla circles of link l1,l2,l3
%% and so on

%% @CollisionVarCritRadiusR, the set of critical circles counting form the right anchor [l_n,0]

%% @pt is a vertex of a convex hull that covers a given original obstacle

%% @intv, the orientation interval for which the robot doesn't potentially intersect with the obstacle

%% @j  is the index of the link  (j> 2 and j < DOF-2) for which it collide with obstacle

%% @nobst_samples,  number of samples to generate 

%% initialize some variables
DOF = length(linklength);
newConfig = [];

%% check the feasibility of this function, basically this function try to randomly sample a configuration around obstacle
%% for a middle piece of the closed chain
if start_link_index <= 2 | start_link_index >= DOF-2
    fprintf(1,' the link index input in MidLinkRandomLoopGenerator should be > 2 and less than DOF-2 \n');
    return;
end

clink = linklength(start_link_index);  %% target collision link
intv =  angle_intv(:,start_vert_index);
numSample = 0;
for ddd=1:nobst_samples,
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
  initAngle = atan2(pt1(2),pt1(1));
    
  leftConfig = [];
  rightConfig = [];
  b_sam = [];
  [leftConfig,b_sam] = RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,false,1,initAngle);
  leftConfig = [leftConfig, b_sam];
  numLeft = size(leftConfig,2);

  %% only when leftConfig is not empty, then we further investigate the right half of the closed chain
  if numLeft > 0
     leftConfig = leftConfig(1:start_link_index-1,:);  
     leftEnd = pt2;  
     rightEnd = [linklength(DOF),0]';
     ccw = true;
     rightConfig1 = [];
     [rightConfig1] = TraceObstHomotopy(linklength, CollisionVarCritRadiusL,CollisionVarCritRadiusR, 1,...
                                           vert, angle_intv, ...
                                           start_link_index-1, start_vert_index, leftEnd,rightEnd, peri, ccw);
    
     rightConfig = [rightConfig,rightConfig1];
                                       
                                                      
     ccw = false;
     rightConfig1 = [];
     [rightConfig1] = TraceObstHomotopy(linklength, CollisionVarCritRadiusL,CollisionVarCritRadiusR, 1,...
                                                    vert, angle_intv,...
                                                    start_link_index-1, start_vert_index, leftEnd,rightEnd, peri, ccw);
     rightConfig = [rightConfig, rightConfig1];                       
     numRight = size(rightConfig, 2);
     if numRight > 0
       rightConfig = rightConfig(1:DOF-start_link_index-1,:);
       matOnes = ones(1,numLeft);
       for i=1:numRight,
          rC = rightConfig(:,i);
          rC = [angle+pi;rC;pi];
          matC = [leftConfig;rC * matOnes];
          newConfig(:,numSample+1:numSample+numLeft) = matC(:,1:numLeft);
          numSample = numSample + numLeft;
       end
     end
  end
end