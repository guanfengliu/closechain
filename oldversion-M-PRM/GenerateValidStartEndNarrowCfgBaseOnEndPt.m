function [start_config,end_config]=GenerateValidStartEndNarrowCfgBaseOnEndPt(mpData,pos_left,pos_right,j_start,j_end, numPair)

linklength = mpData.linklength; 
obst = mpData.obst;
thickNess = mpData.thickNess;
polyObst = mpData.polyObst;
%% @brief this is the function to generate start and goal configs passing through narrow passages
%% a closed chain based at (0,0), while ended at (lm,0); 
%% @param linklength is a length vector of ordered chain
%% @param thinkNess, the thinckness of links for collision checking
%% @CollisionVarCritRadiusL, the data structure such that CollisionVarCritRadiusL(i).linklength represents the radii of critical circles from link0 upto link i
%% @CollisionVarCritRadiusR, same as above, but calculated from right to left
%% @polyObst, represents the obstacles as polygonal, for collisoin checking
%% @obst, the structure representing all obstacles, including point, convex, or non-convex obstacles 
%% @pos_left and j_start, the ending point of link(j_start) for start conformation; this requires
%% @pos_right and j_end, the ending point of link (j_end) for end_conformation
%% Author: Leon G.F. Liu  09/23/2019
MAX_FAILS = 2000;
%% initialization of start and end_config
start_config = [];
end_config = [];
DOF = length(linklength);
if j_start < 2 || j_start > DOF-2 || j_end < 2 || j_end > DOF-2 
    fprintf(1,'link j should be some intermediate link');
    return;
end
%% compute start_config
pos = pos_left;
leftChain = linklength(1:j_start); 
baseLeftLength = norm(pos);
InitAngleL = atan2(pos(2), pos(1));
leftChain = [leftChain, baseLeftLength];
rightChain = linklength(j_start+2:DOF-1);
pos = pos_left + [linklength(j_start+1),0]';
pos_right_w_right = [linklength(DOF),0]' - pos;
baseRightLength = norm(pos_right_w_right);
InitAngleR = atan2(pos_right_w_right(2), pos_right_w_right(1));
rightChain = [rightChain, baseRightLength];

%% Random sample two configuration
collsion = true; %% init in collsion
count = 0;
fail_count = 0;
mpData1 = mpData;
mpData1.linklength = leftChain;
mpData1.enable_bound= false;
mpData1.left_2_right = false;
mpData1.nonbsamples = 1;
mpData1.init_angle = InitAngleL;

mpData2 = mpData;
mpData2.linklength = rightChain;
mpData2.enable_bound= false;
mpData2.left_2_right = true;
mpData2.nonbsamples = 1;
mpData2.init_angle = InitAngleR;
while count < numPair && fail_count < MAX_FAILS
   left_config = [];
   left_config=RandomLoopGeneratorGen2(mpData1);
   right_config = [];
   right_config=RandomLoopGeneratorGen2(mpData2);
   if size(left_config,2) > 0 && size(right_config,2) > 0 %% if generate a valid sample
      sample = [left_config(1:j_start,1);0;right_config(1:DOF-j_start-2,1);pi];  
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
leftChain = linklength(1:j_end); 
baseLeftLength = norm(pos_right);
InitAngleL = atan2(pos_right(2), pos_right(1));
leftChain = [leftChain, baseLeftLength];
rightChain = linklength(j_end+1:DOF-1);
pos_right_w_right = [linklength(DOF),0]' - pos_right;
baseRightLength = norm(pos_right_w_right);
InitAngleR = atan2(pos_right_w_right(2), pos_right_w_right(1));
rightChain = [rightChain, baseRightLength];

mpData1.linklength = leftChain;
mpData1.init_angle = InitAngleL;

mpData2.linklength = rightChain;
mpData2.init_angle = InitAngleR;

while count < numPair && fail_count < MAX_FAILS
   left_config = [];
   left_config=RandomLoopGeneratorGen2(mpData1);
   right_config = [];
   right_config=RandomLoopGeneratorGen2(mpData2);
   if size(left_config,2) > 0 && size(right_config,2) > 0 %% if generate a valid sample
      sample = [left_config(1:j_end,1);right_config(1:DOF-j_end-1,1);pi];  
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