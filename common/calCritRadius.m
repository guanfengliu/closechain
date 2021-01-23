function [CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(linklength)
%% given linklength vector of a closed chain, calculate the set of critical circles from left and from right

num_link = length(linklength);
oldLengthVec=[linklength(1)];
CollisionVarCritRadiusL = [];
CollisionVarCritRadiusR = [];
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