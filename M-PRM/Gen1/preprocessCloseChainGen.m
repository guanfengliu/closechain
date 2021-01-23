function [CollisionVarCritRadiusL,CollisionVarCritRadiusR,poly_dilate,polyObst] = preprocessCloseChainGen(linklength,obst,dilate_epsilon)
%% this function is for preprocessing some data structures based upon linklength vector and vector of point obstacles
%% moreover, it also print out the point obstacles, the dilated point obstacle set
%% @param linklength, a vector linklengths
%% @param obst, 2 by n matrix with each column the original point obstacle
%% @output1 CollisionVarCritRadiusL, the structure for recording the radius of a recursive set of critical circles
%% e.g.CollisionVarCritRadiusL(1) = {abs(l1-l2), l1+l2}
%% @output2 dilatePtobst, the set of point obstacles that samples the boundary of a dilated obstacle
%% @output3 polyObst, a data structure that records the vertices and faces of all slightly dilated obstacles, mainly for
%% collision checking
num_link=length(linklength);  %% number of links
if num_link < 4
    fprintf(1,'C-space is not a continuous space if number of links £¨including base) < 4 \n');
    return;
end
num_obsz=size(obst,2);  %% number of obstacle

%% need to generate the structure of boundary variety
%% Recall B(1)= f1^{-1}(l1+l2 circle + abs(l1-l2) circle)
%% recursively, we can deduce B(2) = f2^{-1}£¨l1+l2+l3 circle, l1+l2-l3 circle, l1-l2+l3 circle,
%% l1-l2-l3 circle)
%% data structure for storing B(1) ... B(n-3) critical circle lengths

oldLengthVec=[linklength(1)];
%% CollionVarCritRadiusL.o_linklength is the radii of original critical circles
%% CollisionVarCritRadiusL.linklength(1) sometimes could be 0, which might be different
%% from CollisionVarCritRadiusL.o_linklength(1)
for i=2:num_link-1
    newLengthVec=[abs(oldLengthVec-linklength(i)),oldLengthVec + linklength(i)];
    CollisionVarCritRadiusL(i-1).o_linklength=sort(newLengthVec);
    oldLengthVec=CollisionVarCritRadiusL(i-1).o_linklength;
end
for i=2:num_link-1
    CollisionVarCritRadiusL(i-1).linklength = CollisionVarCritRadiusL(i-1).o_linklength;
    [b_reach_origin] = check_reach_origin(linklength(1:i));
    if b_reach_origin
        CollisionVarCritRadiusL(i-1).linklength(1)= 0;
    end
end
oldLengthVec=[linklength(num_link-1)];
for i=num_link-2:-1:1
    newLengthVec=[abs(oldLengthVec-linklength(i)),oldLengthVec + linklength(i)];
    CollisionVarCritRadiusR(num_link-1-i).o_linklength=sort(newLengthVec);  
    oldLengthVec=CollisionVarCritRadiusR(num_link-1-i).o_linklength;
end
for i=num_link-2:-1:1
    [b_reach_origin] = check_reach_origin(linklength(i:num_link-1));
    CollisionVarCritRadiusR(num_link-1-i).linklength = CollisionVarCritRadiusR(num_link-1-i).o_linklength;
    if b_reach_origin
        CollisionVarCritRadiusR(num_link-1-i).linklength(1)= 0;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[poly_dilate, polyObst] = processObstacle(obst,dilate_epsilon);