function [out, b_samples] = RandomLoopGeneratorGen2(mpData)
%% this function is for generating regular random samples for planar closed chains (or spatial closed chains with 
% spherical joints), this is different from J. Cortes's RLG

% @linklength, a vector of link lengths
linklength = mpData.linklength;

% @CollisionVarCritRadiusL, the data structure for describing the set of critical
% circles [l1-l2,l1+l2; l1-l2+l3, l1-l2-l3,l1+l2-l3,l1+l2+l3; ......]
CollisionVarCritRadiusL = mpData.CollisionVarCritRadiusL;


% @CollisionVarCritRadiusR, the data structure for the set of critical circles calculated
% base at right anchor
CollisionVarCritRadiusR = mpData.CollisionVarCritRadiusR;

% @enable_bound, do we also smaple the boundary variety
enable_bound = mpData.enable_bound;

% @left_2_right, true, we use CollisionVarCritRadiusR to compute the range of
%joint angles; false, we use CollisionVarCritRadiusL to compute the range
%of joint angles;  the reason why we use this flag is that we want to reuse
%the data structure of CollisionVarCritRadiusL, CollisionVarCritRadiusR
left_2_right = mpData.left_2_right;

% @n_samples, the number of regular samples to be gerenated that satisfies the
% loop-closure constraints
n_samples = mpData.nonbsamples;  %%mpData.nobst_samples
% @initAngle, the orientation angle of the virtual base link; this angle
% will be added to the angles generated for the canonical loops
% (i.e.initAngle=0)
initAngle = mpData.init_angle;

%% Author: Leon G.F. Liu  09/23/2019
out=[];
b_samples=[];
DOF=length(linklength); %size(linklength,2);

%% first check whether the linklengths have non-empty solution
[b_reach_origin] = check_reach_origin(linklength);
if ~b_reach_origin
    %%fprintf(1,'C-space is empty with the given linklengths \n');
    return;
end
%fprintf(1,'enablebound=%d',enable_bound);
%%enable_bound=false; % disable boundary sampling
%% start interation
count=0;  %% count for non-bound samples
b_count=0; %% count for bound samples
if DOF == 3 %% because at this case, there are only at most two solutions for the close chain
    n_samples = 1;
end

CritRadii=[];
limit_in_order = true;
if left_2_right,  %% means sample theta1, 
    LLIMIT = 1;
    RLIMIT = DOF-3;
    if RLIMIT < LLIMIT
        limit_in_order = false;
    end
    step = 1;
    CritRadii = CollisionVarCritRadiusR;
    ind_base = DOF-1;
    ind_up = DOF-2;
else
    LLIMIT = DOF-1;
    RLIMIT = 3;
    if LLIMIT < RLIMIT
        limit_in_order = false;
    end
    step = -1;
    CritRadii = CollisionVarCritRadiusL;
    ind_base = 1;
    ind_up = 2;
end
while count < n_samples
    horizon_angle=0;
    rightEnd=[linklength(DOF),0];
    rightEnd_old=rightEnd; %for sampling boundary variety
    leftEnd = [0,0];
    leftEnd_old = leftEnd;

    x=[];  %% initialize joint angle vector
    x(DOF)=pi;
    % first step:sample non-boundary points
    if limit_in_order
      for i=LLIMIT:step:RLIMIT,
        diffEnd = rightEnd - leftEnd;  
        l1= norm(diffEnd); %(rightEnd(1)^2+rightEnd(2)^2)^0.5;
        horizon_angle=atan2(diffEnd(2),diffEnd(1));
        if left_2_right,
          ind = DOF-2-i;
        else
          ind = i-2;
        end
        critic=CritRadii(ind).linklength;
        dim=size(critic,2);
        lc=critic(dim); % largest one
        lc_min=critic(1); % smallest one
        l2=linklength(i);
      
        %% check if (l1,lc or lc_min,l2)  can form a triangle
        bigcirc=false;
        if l1<=l2+lc && l1>=abs(l2-lc)
           alpha=acos((l1^2+l2^2-lc^2)/(2*l1*l2));
           tmax=alpha;
           tmin=-alpha;
           bigcirc=true;
        end
        smallcirc=false;
        if l1<=l2+lc_min && l1>=abs(l2-lc_min)
           alpha2=acos((l1^2+l2^2-lc_min^2)/(2*l1*l2));
           tmax2=alpha2;
           tmin2=-alpha2;
           smallcirc=true;
        end
         
        % sampling X(DOF-1)
        if bigcirc 
          if smallcirc
              q=rand(1);  % randome first time to choose upper or lower interval 
              if (q>0.5) 
                  %x(i)=tmin + (tmax-tmin) * rand(1);
                  x(i)=tmax2+(tmax-tmax2)*rand(1);
              else
                  x(i)=tmin+(tmin2-tmin)*rand(1);
              end
          else  % only bigcirc has intersecion
              x(i)=tmin + (tmax-tmin) * rand(1);
          end
        else
          if smallcirc
              q=rand(1);  % randome first time to choose upper or lower interval 
              if (q>0.5) 
                  %x(i)=tmin + (tmax-tmin) * rand(1);
                  x(i)=tmax2+(pi-tmax2)*rand(1);
              else
                  x(i)=tmin2+(-pi-tmin2)*rand(1);
              end
          else
              %always assume there are full circle solution for x(i)
              x(i)=-pi + 2*pi * rand(1);
              %%fprintf('full circle \n');
          end
        end
        x(i)=x(i)+horizon_angle; %% because the bottom link may have an orientation angle
        if left_2_right
          leftEnd_old = leftEnd;
          leftEnd = leftEnd + [l2 * cos(x(i)),l2 * sin(x(i))];
        else
          rightEnd_old=rightEnd;
          rightEnd=rightEnd -[l2 * cos(x(i)),l2 * sin(x(i))]; 
        end
      end
    end
    %%%% theta1, theta2 (the first two joints on the left end)
    diffEnd = rightEnd - leftEnd; 
    l2= norm(diffEnd); %%(rightEnd(1)^2+rightEnd(2)^2)^0.5;
    l1= linklength(ind_base);
    lc = linklength(ind_up);
%     if left_2_right
%         l1 = linklength(DOF-1);
%         lc = linklength(DOF-2);
%     else
%         l1=linklength(1);
%         lc=linklength(2);
%     end
    if l2<=l1+lc && l2>=abs(l1-lc)
      alpha=acos((l1^2+l2^2-lc^2)/(2*l1*l2));
      t=atan2(diffEnd(2),diffEnd(1));
    
      %first solution, elbow up w.r.t. link(ind_base), link(ind_up)
      
      x(ind_base)=t+alpha;
      x(ind_up)=atan2(diffEnd(2)-l1*sin(x(ind_base)),diffEnd(1)-l1*cos(x(ind_base)));
      count=count+1;
      out(:,count)=x';
    
      % second solution
      x(ind_base)=t-alpha;
      x(ind_up)=atan2(diffEnd(2)-l1*sin(x(ind_base)),diffEnd(1)-l1*cos(x(ind_base)));
      count=count+1;
      out(:,count)=x';
    else
        fprintf('no solution \n');
    end
    %% sampling boundary variety, there are two solutions
    if enable_bound && limit_in_order
      y=x;
      if left_2_right,
         l2=linklength(DOF-3);
         ind_1 = DOF-3;
         ind_2 = DOF-2;
         ind_3 = DOF-1;
      else
         l2=linklength(3);
         ind_1 = 3;
         ind_2 = 2;
         ind_3 = 1;
      end
      diffEnd_old = rightEnd_old - leftEnd_old;
      if bigcirc
         y(ind_1)=tmax + horizon_angle;
         diffEnd1=diffEnd_old-[l2*cos(y(ind_1)),l2*sin(y(ind_1))];
         y(ind_2)=atan2(diffEnd1(2),diffEnd1(1));
         y(ind_3)=y(ind_2);
         b_count=b_count+1;
         b_samples(:,b_count)=y';
    
         y(ind_1)=tmin+horizon_angle;
         diffEnd1=diffEnd_old-[l2*cos(y(ind_1)),l2*sin(y(ind_1))];
         y(ind_2)=atan2(diffEnd1(2),diffEnd1(1));
         y(ind_3)=y(ind_2);
         b_count=b_count+1;
         b_samples(:,b_count)=y';
      end
      if smallcirc
        y(ind_1)=tmax2+horizon_angle;
        diffEnd1=diffEnd_old-[l2*cos(y(ind_1)),l2*sin(y(ind_1))];
        if linklength(ind_2)>linklength(ind_3)
           y(ind_2)=atan2(diffEnd1(2),diffEnd1(1));
           y(ind_3)=y(ind_2)-pi;
        else
           y(ind_2)=atan2(diffEnd1(2),diffEnd1(1))-pi;
           y(ind_3)=y(ind_2)+pi; 
        end
        b_count=b_count+1;
        b_samples(:,b_count)=y';
    
        y(ind_1)=tmin2+horizon_angle;
        diffEnd1=diffEnd_old-[l2*cos(y(ind_1)),l2*sin(y(ind_1))];
        if linklength(ind_2)>linklength(ind_3)
           y(ind_2)=atan2(diffEnd1(2),diffEnd1(1));
           y(ind_3)=y(ind_2)-pi;
        else
           y(ind_2)=atan2(diffEnd1(2),diffEnd1(1)) - pi;
           y(ind_3)=y(ind_2) + pi; 
        end
        b_count=b_count+1;
        b_samples(:,b_count)=y'; 
      end
    end
end
if b_count > 0
   b_samples = b_samples + initAngle;
end
if count > 0
  out = out + initAngle;
end