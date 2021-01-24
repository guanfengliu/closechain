function [start_config,end_config]=GenerateValidStartEndNarrowCfg(mpData,pos_left,pos_right,j_start,j_end, j_start2, j_end2, numPair)
%% @brief this is the function to generate start and goal configs passing through narrow passages
%% formed by two pairs of obstacles
%% mpData.linklength, the linklength vector of a closed chain based at (0,0), while ended at (lm,0);
%% our algorithm for generating such start and goal config by let a fragment of closed chain passing through
%% both pair of obstacles, and then closed the loop from left and right

%% @param linklength is a length vector of ordered chain
linklength = mpData.linklength; 

%% @param obstacle for drawing config and obstacles
obst = mpData.obst;

%% @param thinkNess, the thinckness of links for collision checking
thickNess = mpData.thickNess;

%% @polyObst, represents the obstacles as polygonal, for collisoin checking
polyObst = mpData.polyObst;

%% With @(pose_left, pose_right)  and @(j_start, j_end)  the chain is divided into three closed subchains
%% Chain 1: (1,.., j_start-1) with end points,  [0,0] and pose_left
%% chain 2: (j_start,..., j_end) with end points pose_left and pose_right
%% chain 3: (j_end+1,..., m) with end points, pose_right and [l_m,0];
%% @pos_left and j_start, the ending point of link(j_start) for start conformation; this requires
%% @pos_right and j_end, the ending point of link (j_end) for end_conformation
%% similalry,  j_start2, j_end2 is the link indices for goal configuration generation
%% Author: Leon G.F. Liu  09/23/2019
MAX_FAILS = 2000;
%% initialization of start and end_config
start_config = [];
end_config = [];
DOF = length(linklength);
if j_start < 3 || j_start > j_end || j_end > DOF-3 ...
    || j_start2 < 3 || j_start2 > j_end2 || j_end2 > DOF-3     
    fprintf(1,'link j_start or j_end is wrong');
    return;
end
%% compute start_config
diffPos1 = pos_left;
leftChain = linklength(1:j_start-1); 
baseLeftLength = norm(diffPos1);
InitAngleL = atan2(diffPos1(2), diffPos1(1));
leftChain = [leftChain, baseLeftLength];

rightChain = linklength(j_end+1:DOF-1);
diffPos2 = [linklength(DOF),0]' - pos_right;
baseRightLength = norm(diffPos2);
InitAngleR = atan2(diffPos2(2), diffPos2(1));
rightChain = [rightChain, baseRightLength];

midChain = linklength(j_start:j_end);
diffPos3 = pos_right - pos_left;
baseMidLength = norm(diffPos3);
midChain = [midChain, baseMidLength];
InitAngleM = atan2(diffPos3(2), diffPos3(1));

[CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(midChain);

%% Random sample two configuration
collsion = true; %% init in collsion
count = 0;
fail_count = 0;
mpData1 = mpData;
mpData1.linklength = leftChain;
mpData1.enable_bound= true;
mpData1.left_2_right = false;
mpData1.nonbsamples = 1;
mpData1.init_angle = InitAngleL;

mpData2 = mpData;
mpData2.linklength = rightChain;
mpData2.enable_bound= false;
mpData2.left_2_right = true;
mpData2.nonbsamples = 1;
mpData2.init_angle = InitAngleR;

mpData3 = mpData;
mpData3.linklength = midChain;
mpData3.enable_bound = false;
mpData3.left_2_right = false;
mpData3.nonbsamples = 1;
mpData3.init_angle = InitAngleM;
mpData3.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
mpData3.CollisionVarCritRadiusR = CollisionVarCritRadiusR;

while count < numPair && fail_count < MAX_FAILS
   mid_config = [];
   mid_config = RandomLoopGeneratorGen2(mpData3);
   left_config = [];
   b_sample = [];
   [left_config, b_sample]=RandomLoopGeneratorGen2(mpData1);
   left_config = [left_config, b_sample];
   right_config = [];
   right_config=RandomLoopGeneratorGen2(mpData2);
   if size(left_config,2) > 0 &&  size(mid_config,2) > 0 && size(right_config,2) > 0 %% if generate a valid sample
      sample = [left_config(1:j_start-1,1);mid_config(1:j_end-j_start+1,1);right_config(1:DOF-j_end-1,1);pi];  
      drawConfig(linklength, sample, obst, 0.01);
      fv = closedchainthick (linklength,sample,thickNess);
      if (~CollisionCheck(fv, polyObst))
         count = count + 1;
         start_config(:,count) = sample;
      else
         fail_count = fail_count + 1;
      end
   else
       fail_count = fail_count + 1;
   end
end
%% for end_config
count = 0;
fail_count = 0;

diffPos1 = pos_left;
leftChain = linklength(1:j_start2-1); 
baseLeftLength = norm(diffPos1);
InitAngleL = atan2(diffPos1(2), diffPos1(1));
leftChain = [leftChain, baseLeftLength];

rightChain = linklength(j_end2+1:DOF-1);
diffPos2 = [linklength(DOF),0]' - pos_right;
baseRightLength = norm(diffPos2);
InitAngleR = atan2(diffPos2(2), diffPos2(1));
rightChain = [rightChain, baseRightLength];

midChain = linklength(j_start2:j_end2);
diffPos3 = pos_right - pos_left;
baseMidLength = norm(diffPos3);
midChain = [midChain, baseMidLength];
InitAngleM = atan2(diffPos3(2), diffPos3(1));

[CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(midChain);

mpData1.linklength = leftChain;
mpData1.init_angle = InitAngleL;

mpData2.linklength = rightChain;
mpData2.init_angle = InitAngleR;

mpData3.linklength = midChain;
mpData3.init_angle = InitAngleM;
mpData3.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
mpData3.CollisionVarCritRadiusR = CollisionVarCritRadiusR;

while count < numPair && fail_count < MAX_FAILS
   mid_config = [];
   mid_config = RandomLoopGeneratorGen2(mpData3); 
   left_config = []; b_sample = [];
   [left_config, b_sample] =RandomLoopGeneratorGen2(mpData1);
   left_config = [left_config, b_sample];
   right_config = [];
   right_config=RandomLoopGeneratorGen2(mpData2);
   if size(left_config,2) > 0 && size(mid_config,2) > 0 && size(right_config,2) > 0 %% if generate a valid sample
      sample = [left_config(1:j_start2-1,1);mid_config(1:j_end2-j_start2+1,1);right_config(1:DOF-j_end2-1,1);pi];
      drawConfig(linklength, sample, obst, 0.01);
      fv = closedchainthick (linklength,sample,thickNess);
      if (~CollisionCheck(fv, polyObst))
         count = count + 1;
         end_config(:,count) = sample;
      else
         fail_count = fail_count + 1;
      end
   else
       fail_count = fail_count + 1;
   end
end