function out=RandomLoopGenerator(linklength,CollisionVarCritRadiusL,nsamples,initAngle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% modular function for random loop configuration sampling, this is
% different from T. Siemon's RLG
% @linklength, a vector of link lengths
% @CollisionVarCritRadiusL, the datastructure for describing the set of critical
% circles [l1-l2,l1+l2; l1-l2+l3, l1-l2-l3,l1+l2-l3,l1+l2+l3; ......]
% @nsamples, the number of samples to be gerenated that satisfies the
% loop-closure constraints
% @initAngle, the orientation angle of the virtual base link; this angle
% will be added to the angles generated for the canonical loops
% (i.e.initAngle=0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Leon G.F. Liu  09/23/2019

DOF=size(linklength,2);

%% first check whether the linklengths have non-empty solutions
critRadVec=CollisionVarCritRadiusL(DOF-2).linklength;
szCrit = length(critRadVec);
maxCritRad = critRadVec(szCrit);
minCritRad = critRadVec(1);
if linklength(DOF) > maxCritRad | linklength(DOF) < minCritRad
    fprintf(1,'C-space is empty with the given linklengths \n');
    return;
end

%% start interation
count=0;
while count < nsamples
    x(DOF)=pi;
    horizon_angle=0;
    rightEnd=[linklength(DOF),0];
    rightEndOld=rightEnd; %for sampling boundary variety
    % first step:sample non-boundary points
    for i=DOF-1:-1:3
      l1=(rightEnd(1)^2+rightEnd(2)^2)^0.5;
      horizon_angle=atan2(rightEnd(2),rightEnd(1));
      critic=CollisionVarCritRadiusL(i-2).linklength;
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
          end
      end
      x(i)=x(i)+horizon_angle; %% because the bottom link may have an orientation angle
      rightEnd_old=rightEnd;
      rightEnd=rightEnd -[l2 * cos(x(i)),l2 * sin(x(i))]; 
    end
    %%%% theta1, theta2 (the first two joints on the left end)
    l2=(rightEnd(1)^2+rightEnd(2)^2)^0.5;
    l1=linklength(1);
    lc=linklength(2);
    alpha=acos((l1^2+l2^2-lc^2)/(2*l1*l2));
    t=atan2(rightEnd(2),rightEnd(1));
    
    %first solution
    x(1)=t+alpha;
    x(2)=atan2(rightEnd(2)-l1*sin(x(1)),rightEnd(1)-l1*cos(x(1)));
    count=count+1;
    out(:,count)=x';
    
    % second solution
    x(1)=t-alpha;
    x(2)=atan2(rightEnd(2)-l1*sin(x(1)),rightEnd(1)-l1*cos(x(1)));
    count=count+1;
    out(:,count)=x';
    %%%% sampling boundary variety, there are two solutions
    y=x;
    l2=linklength(3);
    if bigcirc
      y(3)=tmax + horizon_angle;
      rightEnd1=rightEnd_old-[l2*cos(y(3)),l2*sin(y(3))];
      y(2)=atan2(rightEnd1(2),rightEnd1(1));
      y(1)=y(2);
      count=count+1;
      out(:,count)=y';
    
      y(3)=tmin+horizon_angle;
      rightEnd1=rightEnd_old-[l2*cos(y(3)),l2*sin(y(3))];
      y(2)=atan2(rightEnd1(2),rightEnd1(1));
      y(1)=y(2);
      count=count+1;
      out(:,count)=y';
    end
    if smallcirc
      y(3)=tmax2+horizon_angle;
      rightEnd1=rightEnd_old-[l2*cos(y(3)),l2*sin(y(3))];
      if linklength(2)>linklength(1)
         y(2)=atan2(rightEnd1(2),rightEnd1(1));
         y(1)=y(2)-pi;
      else
         y(1)=atan2(rightEnd1(2),rightEnd1(1));
         y(2)=y(1)-pi; 
      end
      count=count+1;
      out(:,count)=y';
    
      y(3)=tmin2+horizon_angle;
      rightEnd1=rightEnd_old-[l2*cos(y(3)),l2*sin(y(3))];
      if linklength(2)>linklength(1)
         y(2)=atan2(rightEnd1(2),rightEnd1(1));
         y(1)=y(2)-pi;
      else
         y(1)=atan2(rightEnd1(2),rightEnd1(1));
         y(2)=y(1)-pi; 
      end
      count=count+1;
      out(:,count)=y'; 
    end
end
out = out + initAngle;