function [interv] = cal_interval(l1,l2,lc,lc_min,base_angle)
%% this function checks if l1, l2, lc can form a trianle, with base linklength equal to l1
%% the other two link lengths are (l2,lc).  The orientation angle of l1 is base_angle
epsilon = 0.001;
bigcirc1=false;
if l1<=l2+lc + epsilon && l1>=abs(l2-lc) -epsilon
   tt =  (l1^2+l2^2-lc^2)/(2*l1*l2);
   if abs(tt-1) < epsilon 
       alpha = 0;
   else
      if abs(tt+1) < epsilon
         alpha = pi;
      else
         alpha = acos(tt);
      end
   end
   tmax=alpha + base_angle;
   tmin=-alpha + base_angle;
   bigcirc1=true;
end
smallcirc1=false;
if l1<=l2+lc_min + epsilon && l1>=abs(l2-lc_min)-epsilon
   tt = (l1^2+l2^2-lc_min^2)/(2*l1*l2);
   if abs(tt-1) < epsilon 
      alpha2 = 0;
   else
      if abs(tt+1) < epsilon
         alpha2 = pi;
      else
         alpha2 = acos(tt);
      end
   end
   tmax2=alpha2 + base_angle;
   tmin2=-alpha2 + base_angle;
   smallcirc1=true;
end
if bigcirc1 
   if smallcirc1
      interv(:, 1) = [tmax2, tmax]';
      interv(:, 2) = [tmin, tmin2]';
   else
      interv(:, 1) = [tmin, tmax]';
   end
else
   if smallcirc1
      interv(:, 1) = [tmax2, pi]'; 
      interv(:, 2) = [-pi, tmin2]';
   else
      interv(:,1) = [-pi, pi]';
   end
end