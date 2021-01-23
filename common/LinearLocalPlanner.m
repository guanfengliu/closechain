function [traj_out_j, dist] = LinearLocalPlanner(x1, x2, step_delta)
%% simple local planner (linear interpolation) between two given robot configurations x1, x2

% use one constant parameter, step_delta=1 degree
delta = ConvertNormal(x2-x1);
dist = norm(delta); %sum(abs(deltanew));
if dist < 0.0001
   traj_out_j = [x1,x2];
else
   deltanew = delta/dist;  % normalize deltanew;
   dist_vec = 0:step_delta:dist;
   traj_out_j = x1 + deltanew * dist_vec;
end