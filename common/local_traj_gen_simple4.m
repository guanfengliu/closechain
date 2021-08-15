function phi_traj = local_traj_gen_simple4(x1,deltanew,nsteps, sign1, linklength)
%% @brief: this function is to generate a list of samples of configurations for a n-link closed chain
%% @x1: the start configuration 
%% @deltanew: the step added to x1,  i.e. x1(3,DOF-1) + i * deltanew gives the configuraiton at ith step of the last DOF-3 joints
%% @nsteps: the number of steps for iteration
%% @sign1: 1, means phi_traj(2,:) - phi_traj(1,:) has to be positive (CCW), 0, CW
%% @linklength: the vector of link lengths of the entire closed chain

DOF = length(linklength);
startAng=x1(1:DOF-3);
delta=deltanew/nsteps;
lastlink=linklength(DOF);
links=linklength(1:DOF-3);
phi_traj=[];
for i=1:nsteps,
    midAngle=startAng + i * delta;
    x=sum(links.* cos(midAngle));
    y=sum(links.* sin(midAngle));
    newVec=[lastlink-x,-y]';
    centAng=atan2(newVec(2),newVec(1));
    lc=norm(newVec);
    l1=linklength(DOF-2);
    l2=linklength(DOF-1);
    epsilon = 0;
    if (l1+l2< lc-epsilon) || (abs(l1-l2)> lc+epsilon)
        phi_traj=[];
        return;
    end
    alpha=abs(acos((l1^2+lc^2-l2^2)/(2*l1*lc)));
    if sign1==1
        t1 = centAng-alpha;
    else
         t1=centAng+alpha;
    end
    t2=atan2(newVec(2)-l1*sin(t1),newVec(1)-l1*cos(t1));
    phi_traj(:,i)=[midAngle; t1;t2;pi];
end