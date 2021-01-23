function [mpData_out] = preprocessCloseChainGen(mpData)
%% this function is for preprocessing some data structures based upon linklength vector and vector of point obstacles
%% moreover, it also print out the point obstacles, the dilated point obstacle set
%% @param linklength, a vector linklengths
%% @param obst, 2 by n matrix with each column the original point obstacle
%% @output1 CollisionVarCritRadiusL, the structure for recording the radius of a recursive set of critical circles
%% e.g.CollisionVarCritRadiusL(1) = {abs(l1-l2), l1+l2}
%% @output2 dilatePtobst, the set of point obstacles that samples the boundary of a dilated obstacle
%% @output3 polyObst, a data structure that records the vertices and faces of all slightly dilated obstacles, mainly for
%% collision checking
mpData_out = mpData;
linklength = mpData.linklength;
obst = mpData.obst;
dilate_epsilon = mpData.dilate_epsilon;

num_link=length(linklength);  %% number of links
if num_link < 4
    fprintf(1,'C-space is not a continuous space if number of links including base < 4 \n');
    return;
end
%% need to generate the structure of boundary variety
%% Recall B(1)= f1^{-1}(l1+l2 circle + abs(l1-l2) circle)
%% recursively, we can deduce B(2) = f2^{-1}£¨l1+l2+l3 circle, l1+l2-l3 circle, l1-l2+l3 circle,
%% l1-l2-l3 circle)
%% data structure for storing B(1) ... B(n-3) critical circle lengths
[CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(linklength);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[poly_dilate, polyObst] = processObstacle(obst,dilate_epsilon);
mpData_out.CollisionVarCritRadiusL = CollisionVarCritRadiusL;  %% set of critical circles counting from left anchor
mpData_out.CollisionVarCritRadiusR = CollisionVarCritRadiusR;  %% set of critical circles counting from right anchor
mpData_out.poly_dilate = poly_dilate;  %% enveloping polygon for generating compatible configurations
mpData_out.polyObst = polyObst;  %% actual polygonal obstacles for collision checking