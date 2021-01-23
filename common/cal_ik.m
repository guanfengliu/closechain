function [interv] = cal_ik(l2,lc,pr)
%% this function checks if l1, l2, lc can form a trianle, with base linklength equal to l1
%% the other two link lengths are (l2,lc).  The orientation angle of l1 is base_angle
%% @param l1 base link
%% @param l2, the right link with anchor point pr
%% @param lc, the left link with anchor point [0,0]
%% @param pr, the right anchor point

epsilon = 0;
l1 = norm(pr);
if l1 <= l2 + lc + epsilon && l1>= abs(l2-lc) -epsilon
    tt =  (l1^2+l2^2-lc^2)/(2*l1*l2);
%    if abs(tt-1) < epsilon 
%        alpha = 0;
%    else
%       if abs(tt+1) < epsilon
%          alpha = pi;
%       else
%          alpha = acos(tt);
%       end
%    end
   alpha = acos(tt);
   base_angle = atan2(pr(2),pr(1));
   tmax=alpha + base_angle;
   tmin=-alpha + base_angle;
   pr1 = pr - [l2 * cos(tmax), l2 * sin(tmax)]';
   interv(:,1) = [atan2(pr1(2),pr1(1)), tmax]';   %% elbow up
   pr1 = pr - [l2 * cos(tmin), l2 * sin(tmin)]';
   interv(:,2) = [atan2(pr1(2),pr1(1)), tmin]';   %% elbow down
else
   interv =[];
end