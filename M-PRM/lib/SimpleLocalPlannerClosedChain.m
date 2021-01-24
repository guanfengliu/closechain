function [out,traj_out,dist] = SimpleLocalPlannerClosedChain(x1, x2, mpData)
obstacle = mpData.polyObst;
linklength = mpData.linklength;
thickness = mpData.thickNess;
%% Generate a random freespace configuration for the robot
% LocalPlanner has 5 inputs
%@x1: target configuration
%@x2: closest configuration on current roadmap
% in fact x1,x2 could be any two different robot configurations
%@obstacle: triangulated obstacles (with vertices and faces structures)
%@linklength: a vector of link lengths (including  DOF) 
%% Author: Leon G.F. Liu  09/23/2019

%% first check  if  sign(x1(2)-x1(1))=sign(x2(2)-x2(1))
epsilon=0.1; %% epsilon=0.02
dx1 = mod(x1(2)-x1(1),2*pi);
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
 
dx2 = mod(x2(2)-x2(1),2*pi);
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
step_delta= 0.04; %%0.02;
DOF=size(linklength,2);
delta = mod(x2 - x1,2*pi);

t = delta > pi;
delta(t) = delta(t) - 2*pi;

t = delta < -pi;
delta(t) = delta(t) + 2*pi;
deltanew=delta(3:DOF-1);  % only 3 to DOF needs to be interpolated, the first two are calculated using IK
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
phi_traj=local_traj_gen_simple(x1,deltanew,nsteps,sign,linklength);
nsamples = size(phi_traj,2);
if (nsamples==0)
    out=false;
    traj_out=phi_traj;
    return;
end

for i = 1:nsamples
    x = phi_traj(:,i); 
    %% collision checking is always w.r.t. the original chain
    fv= closedchainthick(linklength(1:DOF-1),x(1:DOF-1),thickness);
    
    if (CollisionCheck(fv, obstacle))
        out = false;
        traj_out=[];
        return
    end
end

out = true;
traj_out=phi_traj;
