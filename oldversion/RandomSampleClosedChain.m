function out = RandomSampleClosedChain(linklength,thickness,CollisionVarCritRadiusL,CollisionVarCritRadiusR,polyObst,dilatePtObst,nsamples,num_internal_loops)
%% this is a refined version for generating random 
% Generate c-free configurations for an mLink closed-chain
% input: 
% @linklenght=[l_1,l_2,l_3, \cdots, l_{m-1}, l_m], l_m is always
% horizontal, with one end ankured at the origin, and the other end is
% always on postive x-axis. Similarly, one end of l_1 is ankured at the
% origin

%@CollisionVarCritRadiusL=[l1+l2, abs(l1-l2), l1+l2+l3, abs(l1+l2-l3), abs(l1-l2+l3), abs(l1-l2-l3),
%...], which are very important for determining the sampling range of each
%joint angles

%@CollisionVarCritRadiusR£¬ which are caculated based on linklengths from
%right to left

%@polyObst, triangulation of dilated obstacles, mainly for collision
%checking

%@dilatedPtObst, a set of approximating point obstacle for generating
%samples for passing through narrow passages

% this function modified the traditional sampling algorithm in which
% configurations are generated random without considering any structure of
% the robot arm as well as the structure of obstacles; this algorithm uses
% the info of boundary and dilated obstacle varieties for generating mile
% stones

% this function will return a number of samples, which include points from
% boundary variety, and also from collision variety

%first step: RandomLoopGenerator, which already takes care of boundary
%varity as well as sampling points not on the boundary

%% first half: from RandomLoopGenerator, including generating noncritical
% configurations and boundary configurations
samples=RandomLoopGenerator(linklength,CollisionVarCritRadiusL,num_internal_loops * nsamples,0); 
    
%% second step: sampling points on the collision variety
%let link i passing through point i on dilated obstacle set
DOF=size(linklength,2);
numDiaObst=size(dilatePtObst,2);
obstDist=(dilatePtObst(1,:).^2+dilatePtObst(2,:).^2).^0.5;
dilatePtObstR=dilatePtObst - [linklength(DOF),0]';
obstDistR=(dilatePtObstR(1,:).^2 + dilatePtObstR(2,:).^2).^0.5;
angleR=atan2(dilatePtObstR(2,:),dilatePtObstR(1,:));
for i=1:numDiaObst, % check every point obstacle, see if it can be touched by a link
    pt=dilatePtObst(:,i);
    for j=0:DOF-4,
       if j==0
           l_len=0;
           r_len=linklength(1);
       else
           critLengths=CollisionVarCritRadiusL(j).linklength;
           sz=size(critLengths,2);
           r_len=critLengths(sz);
           l_len=critLengths(1);
       end
      
       if obstDist(i) <=r_len && obstDist(i) >= l_len   % meaning obstacle i can be intesected by link j+1
          %%% link 1 ... j, j+1, is equivalent to a closed chain
          %%%  (linklength(1), ..., linklength(j), 0.5 linklength(j+1), 0.5 linkLength(j+1), \|pt\|) 
          %%for k=1:10,  
          leftConfig=[];
          if j==0
              leftConfig= [atan2(pt(2),pt(1))];
              leftEnd=pt * r_len/obstDist(i);
          else 
             if DOF-j-1 >=3  %% right half close-chain has at least two moveable links    
                [leftConfig,leftEnd]=SlideLoopGenerator(linklength(1:j+1),CollisionVarCritRadiusL,pt,nsamples);  %% collision loop closure generator, not this is different from regular loop closure generator
             end
          end
          numConfig=size(leftConfig,2);  %% find out how many configuration have been generated
          for k=1:numConfig,
             newVec=[linklength(DOF),0]'-leftEnd(:,k);
             baseLinkLength=(newVec(1)^2+newVec(2)^2)^0.5;
             initAngle=atan2(newVec(2),newVec(1));
          
             % link j+2 to link DOF forming a new closed-chain
             % the baseLinkLength is also updated as the length of newVec
             newlinklength=[];
             newlinklength(1:DOF-j-2)=linklength(j+2:DOF-1);   %' * ones(1,numConfig)
             newlinklength(DOF-j-1)=baseLinkLength;
             
             dimNewChain = length(newlinklength);
             critRad2 = CollisionVarCritRadiusR(dimNewChain-2).linklength;
             szc=length(critRad2);
             if baseLinkLength <= critRad2(szc) && baseLinkLength>= critRad2(1)
               %% backward random loop generator
                rightConfig=BackWardRandomLoopGenerator(newlinklength,CollisionVarCritRadiusR,nsamples,initAngle);
             
                % merge configure together
                numRight=size(rightConfig,2);
                newConfig=leftConfig(:,k) * ones(1,numRight);
                newConfig=[newConfig;rightConfig];
                newConfig(DOF,:)=pi * ones(1,numRight);  %% the last joint is always pi
                samples=[samples,newConfig];
             end
          end
      end
    end
    %%% two special cases
    for k=1:num_internal_loops    %% num_internal_loops are for increasing the inflation-collision variety samples
      rightConfig=[];
      if obstDistR(i) <= linklength(DOF-1)  %% colliding with link DOF-1
         rightConfig(1)=atan2(dilatePtObstR(2,i),dilatePtObstR(1,i)) - pi;
         rightConfig(2)= -pi;
         pt=[linklength(DOF),0]'+ dilatePtObstR(:,i) *  linklength(DOF-1)/obstDistR(i);
         initAngle=atan2(pt(2),pt(1));
         newlinklength=[];
         newlinklength(1:DOF-2)=linklength(1:DOF-2);
         newlinklength(DOF-1)=norm(pt); 
      end
      rightDOF=size(rightConfig,2);
      if   rightDOF > 0  %means there is a solution for right half chain
         leftConfig=RandomLoopGenerator(newlinklength,CollisionVarCritRadiusL,nsamples,initAngle);
         numLeft=size(leftConfig,2);
         newConfig= rightConfig' * ones(1,numLeft);
         newConfig=[leftConfig(1:DOF-rightDOF,:);newConfig];
         samples=[samples,newConfig];
      end
      rightConfig=[];
      critRadii=CollisionVarCritRadiusR(1).linklength;
      numRadii=size(critRadii,2);
      if obstDistR(i) <= critRadii(numRadii) && obstDistR(i) >= critRadii(1)
            lc=obstDistR(i); %%dilatePtObstR(:,i);
            l1=linklength(DOF-1); 
            l2=linklength(DOF-2);
            alpha=acos((lc^2+l1^2-l2^2)/(2*lc*l1));
            tmax=angleR(i)+alpha;
            tmin=angleR(i)-alpha;
            rightConfig(2)=tmin + rand(1) * (tmax-tmin);
            newVec=dilatePtObstR(:,i)-[l1*cos(rightConfig(2)),l1*sin(rightConfig(2))]';
            rightConfig(1)=atan2(newVec(2),newVec(1));
            pt=[linklength(DOF) + l1 *cos(rightConfig(2)) + l2 * cos(rightConfig(1)), l1 * sin(rightConfig(2)) + l2 * sin(rightConfig(1))]';
            baseLength=norm(pt);
            
            criRad=CollisionVarCritRadiusL(DOF-4).linklength;
            szRad=size(criRad,2);
            maxRad=criRad(szRad);
            minRad=criRad(1);
             
            if baseLength<=maxRad && baseLength>=minRad
               initAngle=atan2(pt(2),pt(1));
               newlinklength=[];
               newlinklength(1:DOF-3)=linklength(1:DOF-3);
               newlinklength(DOF-2)=baseLength;
               rightConfig(3)=0;
              %% recall at this step, rightConfig(1£¬2£© are angles based on the right end, we need to revert back to based on left end
               rightConfig=rightConfig - pi;
            else
               rightConfig=[];
            end     
      end
      rightDOF=size(rightConfig,2);
      if   rightDOF > 0  %means there is a solution for right half chain
         leftConfig=RandomLoopGenerator(newlinklength,CollisionVarCritRadiusL,nsamples,initAngle);
         numLeft=size(leftConfig,2);
         newConfig= rightConfig' * ones(1,numLeft);
         newConfig=[leftConfig(1:DOF-rightDOF,:);newConfig];
         samples=[samples,newConfig];
      end
    end
end
   
%% final step: collision checking
count=0;
num_trials=size(samples,2);

for i=1:num_trials,
    cspace=samples(:,i);
    fv = closedchainthick (linklength,cspace,thickness);
    if (~CollisionCheck(fv, polyObst))
        count=count+1;
        out(:,count)=cspace;
    end
end
