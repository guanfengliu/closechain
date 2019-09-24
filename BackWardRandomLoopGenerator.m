function out=BackWardRandomLoopGenerator(linklength,CollisionVarCritRadiusR,nsamples,initAngle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% modular function for random loop configuration sampling
% @linklength, a vector of link lengths from left to right, 
% @CollisionVarCritRadiusLR, the datastructure for describing the set of critical
% circles [l_{n-1}-l_{n-2},l_{n-1}+l_{n-2};...] from right to left
% @nsamples, the number of samples to be gerenated that satisfies the
% loop-closure constraints
% @initAngle, the orientation angle of the virtual base link; this angle
% will be added to the angles generated for the canonical loops
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Leon G.F. Liu  09/23/2019

DOF=size(linklength,2);

%% first check whether the linklengths have non-empty solutions
critRadVec=CollisionVarCritRadiusR(DOF-2).linklength;
szCrit = length(critRadVec);
maxCritRad = critRadVec(szCrit);
minCritRad = critRadVec(1);
if linklength(DOF) > maxCritRad | linklength(DOF) < minCritRad
    fprintf(1,'C-space is empty with the given linklengths \n');
    return;
end

%% start iteration
count=0;
while count < nsamples
    x(DOF)=pi;
    horizon_angle=0;
    rightEnd=[linklength(DOF),0];
    leftEnd=[0,0];
    leftEndOld=leftEnd;
    % first step:sample non-boundary points
    for i=1:DOF-3
      newVec=rightEnd-leftEnd;  
      l1=(newVec(1)^2+newVec(2)^2)^0.5;
      horizon_angle=atan2(newVec(2),newVec(1));  
      critic=CollisionVarCritRadiusR(DOF-2-i).linklength;
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
      x(i)=x(i)+horizon_angle;            
      leftEnd_old=leftEnd;
      leftEnd=leftEnd + [l2 * cos(x(i)),l2 * sin(x(i))];
    end
    
    newVec=rightEnd-leftEnd;
    l2=(newVec(1)^2+newVec(2)^2)^0.5;
    lc=linklength(DOF-1);
    l1=linklength(DOF-2);
    alpha=acos((l1^2+l2^2-lc^2)/(2*l1*l2));
    t=atan2(newVec(2),newVec(1));
    
    %first solution
    x(DOF-2)=t+alpha;
    x(DOF-1)=atan2(newVec(2)-l1*sin(x(DOF-2)),newVec(1)-l1*cos(x(DOF-2)));
    count=count+1;
    out(:,count)=x';
    
    % second solution
    x(DOF-2)=t-alpha;
    x(DOF-1)=atan2(newVec(2)-l1*sin(x(DOF-2)),newVec(1)-l1*cos(x(DOF-2)));
    count=count+1;
    out(:,count)=x';
    
    %%%% sampling boundary variety, there are two solutions
   if DOF>3
    y=x;
    l2=linklength(DOF-3); %redefine l2
    if bigcirc
      y(DOF-3)=tmax+horizon_angle;
      leftEnd1=leftEnd_old+[l2*cos(y(DOF-3)),l2*sin(y(DOF-3))];
      newVec=rightEnd-leftEnd1;
      y(DOF-2)=atan2(newVec(2),newVec(1));
      y(DOF-1)=y(DOF-2);
      count=count+1;
      out(:,count)=y';
    
      y(DOF-3)=tmin+horizon_angle;
      leftEnd1=leftEnd_old+[l2*cos(y(DOF-3)),l2*sin(y(DOF-3))];
      newVec=rightEnd-leftEnd1;
      y(DOF-2)=atan2(newVec(2),newVec(1));
      y(DOF-1)=y(DOF-2);
      count=count+1;
      out(:,count)=y';
    end
    if smallcirc
      y(DOF-3)=tmax2+horizon_angle;
      leftEnd1=leftEnd_old+[l2*cos(y(DOF-3)),l2*sin(y(DOF-3))];
      newVec=rightEnd-leftEnd1;
      if linklength(DOF-2)>linklength(DOF-1)
         y(DOF-2)=atan2(newVec(2),newVec(1));
         y(DOF-1)=y(DOF-2)-pi;
      else
         y(DOF-1)=atan2(newVec(2),newVec(1));
         y(DOF-2)=y(DOF-1)-pi; 
      end
      count=count+1;
      out(:,count)=y';
    
      y(DOF-3)=tmin2+horizon_angle;
      leftEnd1=leftEnd_old+[l2*cos(y(DOF-3)),l2*sin(y(DOF-3))];
      newVec=rightEnd-leftEnd1;
      if linklength(DOF-2)>linklength(DOF-1)
         y(DOF-2)=atan2(newVec(2),newVec(1));
         y(DOF-1)=y(DOF-2)-pi;
      else
         y(DOF-1)=atan2(newVec(2),newVec(1));
         y(DOF-2)=y(DOF-1)-pi; 
      end
      count=count+1;
      out(:,count)=y'; 
    end
   end
end
out = out + initAngle;