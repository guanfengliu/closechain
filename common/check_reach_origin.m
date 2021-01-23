function [b_reach_origin] = check_reach_origin(linklength)
%% this function checks if an open chain of linklength, whose tip can reach origin
%% this problem is equivalent to whether the close chain of linklength has non-empty C-space.
%% @param linklength, the linklenghth vector of an open chain
DOF=length(linklength);
%% if dof < 3, then reach_origin is always false, unless 2-link case with l1=l2, which
%% will be taken care duing calculation of critical radii
if DOF <= 2
    b_reach_origin=false;
    return;
end
%% using longest link check to see if C-space of the closed chain with lengths=linklength
%% is non-empty, if is, then the distal end of chain can reach origin
longlink = max(linklength);
sumlink = sum(linklength);
if longlink <= 0.5 * sumlink 
    b_reach_origin = true;
else
    b_reach_origin = false;
end
return
