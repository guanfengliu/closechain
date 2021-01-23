function [interv] = cal_ik_2pt(l_base,l_up,pl,pr, index_base, index_up)
%% this function checks if l_base, l_up, nomr(pr-pl) can form a triangle
%% @param l_base, the length of the link (considered as a base link)
% if l_base anchored at pl, then l_up anchored at pr
% if l_base anchored at pr, then l_up anchored at pl
% user needs to input the right link lengths based upon their chains
%% @param l_up, the link next to the based link
%% @param pl, the left anchor point
%% @param pr, the right anchor point
% each colum of interv gives one solution of the IK
% if l_base anchored at pl, interv(:,1) gives elbow-down solution, while
% interv(:,2) gives elbow-up solution
% if l_base anchored at pr, interv(:,1) gives elbow-up soluiton, while
% interv(:,2) gives elbow-down solutoin
if nargin == 4
    index_base = 1;
    index_up = 2;
end
epsilon = 0;
diffvec = pr - pl;

lb = norm(diffvec);
if lb <= l_base + l_up + epsilon && lb >= abs(l_base-l_up) -epsilon
    tt =  (l_base^2+lb^2-l_up^2)/(2*l_base*lb);
    alpha = acos(tt);
    base_angle = atan2(diffvec(2), diffvec(1));
    tmax=alpha + base_angle;
    tmin=-alpha + base_angle;
    pr1 = diffvec - [l_base * cos(tmax), l_base * sin(tmax)]';
    interv(index_base,1)=tmax;
    interv(index_up, 1) = atan2(pr1(2),pr1(1));
    %interv(:,1) = [tmax, atan2(pr1(2),pr1(1))]';   %% elbow up
    pr1 = diffvec - [l_base * cos(tmin), l_base * sin(tmin)]';
    interv(index_base,2)=tmin;
    interv(index_up, 2) = atan2(pr1(2),pr1(1));
    %interv(:,2) = [tmin, atan2(pr1(2),pr1(1))]';   %% elbow down
else
   interv =[];
end