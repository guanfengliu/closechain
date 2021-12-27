function [out, traj_out, dist, numCCs] = LocalPlannerClosedChainSimpleGen4 (x1, x2, mpData)
%% Generate a random freespace configuration for the robot
% LocalPlanner has 3 inputs
%@x1: target configuration
%@x2: closest configuration on current roadmap
% mpData constraints linklength/obstacle and thickness data
%@out: true, local planner succ, false, local planner trapps
%@traj_out, the resulting local path if out=true
%@dist, the length of the path
%@numCCs, number of collision checks
traj_out = [];
%% Author: Leon G.F. Liu  09/23/2019
numCCs = 0;
%% first check  if  sign(x1(2)-x1(1))=sign(x2(2)-x2(1))
epsilon=mpData.conf.epsilon; %% epsilon=0.02
DOF= mpData.num_links;
dx1 = mod(x1(DOF-1)-x1(DOF-2),2*pi);
if dx1 > pi
     dx1 = dx1 - 2*pi;
end
 
if dx1>epsilon && dx1 < pi-epsilon
     sign1=1;  %% ccw
else
    if dx1<-epsilon && dx1> -pi+epsilon
        sign1=-1;
    else
        sign1=0;
    end
end
 
dx2 = mod(x2(DOF-1)-x2(DOF-2),2*pi);
if dx2 > pi
     dx2 = dx2 - 2*pi;
end
 
if dx2>epsilon && dx2 < pi-epsilon
     sign2=1;
else
    if dx2<-epsilon && dx2>-pi+epsilon
        sign2=-1;
    else
        sign2=0;
    end
end
% 
if (abs(sign1-sign2)>1)   % no straightline solution if sign are significantly different; because start and goal belong two different branch,
    % and more over, both are away from the bound variety
     out=false;
     traj_out=[];
     dist = realmax;
     return
end
% use one constant parameter, step_delta=1 degree
step_delta= mpData.conf.step_delta;   %0.04; %%0.02;
DOF= mpData.num_links;   %size(linklength,2);
delta = mod(x2 - x1,2*pi);

t = delta > pi;
delta(t) = delta(t) - 2*pi;

t = delta < -pi;
delta(t) = delta(t) + 2*pi;
deltanew=delta(1:DOF-3);  % only 3 to DOF needs to be interpolated, the first two are calculated using IK
dist = norm(abs(deltanew)); %sum(abs(deltanew));
nsteps= ceil(dist/step_delta);

if sign1==0
    sign=sign2;
else
    sign=sign1;
end

if sign==0 % means both start and goal config are in the boundary variety, then we can arbitray choose sign for calculating IK for the first two link chain
    sign=1;
end

%go through bound penalth
%crossBoundPenalty = 100.0;
%if (sign1 * sign2)==0
%    dist = dist * crossBoundPenalty;
%end
phi_traj=local_traj_gen_simple4(x1,deltanew,nsteps, sign, mpData.conf.linkLengthVec');
nsamples = size(phi_traj,2);
if (nsamples==0)
    out=false;
    %traj_out=phi_traj;
    return;
end

for i = nsamples:-1:1,
    angle = phi_traj(:,i); 
    numCCs = numCCs + 1;
    %% collision checking is always w.r.t. the original chain
    if CheckCollision(angle, mpData, true)
        out = false;
        if i < nsamples
           % return the last free sample
           traj_out= phi_traj(:, i+1: nsamples);
           dist = dist * size(traj_out,2)/nsteps;
        end
        return
    end
end

out = true;
traj_out=phi_traj;
