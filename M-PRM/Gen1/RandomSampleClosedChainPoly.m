function [out, bound_samples, obst_samples] = RandomSampleClosedChainPoly(linklength,thickness,...
         CollisionVarCritRadiusL,CollisionVarCritRadiusR,polyObst,poly_dilate,enable_bound,nonbsamples,nobst_samples)
%% this is a refined version for generating random 
% Generate c-free configurations for an mLink closed-chain
% input: 
%% @linklength, the original link length vector

%%@fCollisionVarCritRadiusL  is the critical radius data structure of
%%floatLinks

%% the following two data structures is the for the original link lengthe vector
%@CollisionVarCritRadiusL=[l1+l2, abs(l1-l2), l1+l2+l3, abs(l1+l2-l3), abs(l1-l2+l3), abs(l1-l2-l3),
%...], which are very important for determining the sampling range of each
%joint angles

%@CollisionVarCritRadiusR£¬ which are caculated based on floatLinks from
%right to left

%@thickness, the thickness of links for collision checking

%@polyObst, triangulation of dilated obstacles, mainly for collision
%checking

%@poly_dilate,
% poly_dilate.numObst,  number of polygonal obstacles including point,
% convex or non-convex point obstacles
% poly_dilate.obst(i).coord,  the set of approximating point obstacles of
% the original obstacle for generating
%samples for passing through narrow passages

%@enable_bound, a boolean for enabling or disable sampling boundary variety

%@nonbsamples, number of boundary variety samples to be generated

%@nobst_samples, number of obstacle_envoloping samples to be generated at
%each envoloping vertex

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
[samples, b_samples]=RandomLoopGeneratorPoly(linklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,enable_bound,false,nonbsamples,0); 

    
%% second step: sampling points on the collision variety
%let link i passing through point i on dilated obstacle set
%% recall collision variety and collision checking is always w.r.t.
%% the original chain
DOF=size(linklength,2);
%% workspace boundry circle radii-structure, for latter sampling use
l_len=[];  % inner crit circle radius
l_len_r = [];  % these are inner crit circles from the right
r_len=[];  % outer crit circle radius
r_len_r = [];  % these are outer crit cricles from the right
for j=0:DOF-2,
   if j==0
      l_len=[l_len,0];
      r_len=[r_len,linklength(1)];
      l_len_r = [l_len_r,0];
      r_len_r = [r_len_r,linklength(DOF-1)];
    else
      critLengths=CollisionVarCritRadiusL(j).linklength;
      sz=size(critLengths,2);
      r_len=[r_len,critLengths(sz)];
      l_len=[l_len,critLengths(1)];
      
      critLengths=CollisionVarCritRadiusR(j).linklength;
      sz=size(critLengths,2);
      r_len_r=[r_len_r,critLengths(sz)];
      l_len_r=[l_len_r,critLengths(1)];
   end
end
numDiaObst= poly_dilate.numObst;   %%size(dilatePtObst,2);

o_samples = []; %% C-obst samples are always based on the original closed chains
rEnd = [linklength(DOF),0]';
for i=1:numDiaObst, % check every point obstacle, see if it can be touched by a link
  vert = poly_dilate.obst(i).coord;
  num_vert = size(vert, 2);
  angle_intv = poly_dilate.obst(i).angle_interval;
  
  vertR = vert - rEnd;
  obstDist = vecnorm(vert);
  obstDistR = vecnorm(vertR);
  
  %% in the following we generate two end points of a link, and then forming closed loop by sampling
  %% two closed chains, one on the left, and the other on the right
  for j=0:DOF-2, % right open chain has at least two links
    for k=1:num_vert,  %iterate through all vertices of each obstacle
        pt = vert(:,k);
        rpt = vertR(:,k);
        intv = angle_intv(:,k);
        if obstDist(k) <= r_len(j+1) && obstDist(k) >= l_len(j+1) ...
           && obstDistR(k) <= r_len(DOF-j-1) && obstDistR(k) >= l_len(DOF-j-1) % meaning obstacle i can be intesected by link j+1
          %%% link 1 ... j, j+1, is equivalent to a closed chain, and the
          %%% remaining links as a second closed chain
          leftConfig=[];
          rightConfig=[];
          in_interval = false;
          newConfig = [];
          if j==0
              angle= atan2(-pt(2),-pt(1));
              in_interval = CheckAngleInInterval(angle, intv);
              leftConfig = angle + pi;
              if in_interval
                leftEnd=pt * r_len(j+1)/obstDist(k);
                newVec=rEnd-leftEnd;
              end
          else
              if j==1   %% link 2 touch the obstacle
               %% compute the intersection of the circle of l1 based at (0,0) and the circle of l2 based at pt
                 tc1 = atan2(pt(2),pt(1));
                 alpha1 = acos((linklength(1)^2 + obstDist(k)^2  - linklength(2)^2)/(2 * linklength(1) * obstDist(k)));
                 tup1 = tc1 + alpha1;
                 tdown1 = tc1 - alpha1;
                 t1 = tdown1 + rand(1) * (tup1 - tdown1);
                 dist_end1 = [linklength(1) * cos(t1), linklength(1) * sin(t1)]';
                 vec1 = dist_end1 - pt;
                 angle = atan2(vec1(2), vec1(1));
                 in_interval =  CheckAngleInInterval(angle, intv);
                 if in_interval
                     leftConfig = [t1,angle + pi]';
                     leftEnd =(sum([linklength(1),linklength(2)]' .* [cos(leftConfig),sin(leftConfig)]))';
                     newVec=rEnd-leftEnd;
                 end
              else
                 if j == DOF -3   %% link DOF-2 touch the obstacle
                     tc1 = atan2(rpt(2),rpt(1));
                     alpha1 = acos((linklength(DOF-1)^2 + obstDistR(k)^2  - linklength(DOF-2)^2)/(2 * linklength(DOF-1) * obstDistR(k)));
                     tup1 = tc1 + alpha1;
                     tdown1 = tc1 - alpha1;
                     t1 = tdown1 + rand(1) * (tup1 - tdown1);
                     dist_end1 = [linklength(DOF-1) * cos(t1), linklength(DOF-1) * sin(t1)]';
                     vec1 = dist_end1 - rpt;
                     angle = atan2(vec1(2), vec1(1));
                     in_interval =  CheckAngleInInterval(angle, intv);
                     if in_interval
                        rightConfig = [angle, t1+pi]';
                        newVec =rEnd - (sum([linklength(DOF-2),linklength(DOF-1)]' .* [cos(rightConfig),sin(rightConfig)]))';
                     end
                 else  
                     if j==DOF-2  %% link DOF-1 touch the obstacle
                        rightConfig= atan2(-rpt(2),-rpt(1));
                        in_interval = CheckAngleInInterval(rightConfig, intv);
                        if in_interval
                           newVec=rEnd - linklength(DOF-1) * [cos(rightConfig), sin(rightConfig)]';;
                        end
                     else
                        [newConfig]=MidLinkRandomLoopGeneratorOld(linklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR, pt, intv, j+1, nobst_samples);  %% collision loop closure generator, not this is different from regular loop closure generator
                     end
                 end
              end
          end
          if in_interval,
             baseLinkLength=norm(newVec);
             initAngle=atan2(newVec(2),newVec(1)); 
             if j==0 | j==1
                % link j+2 to link DOF forming a new closed-chain
                % the baseLinkLength is also updated as the length of newVec
                newlinklength=[];
                newlinklength(1:DOF-j-2)=linklength(j+2:DOF-1);   %' * ones(1,numConfig)
                newlinklength(DOF-j-1)=baseLinkLength;
                b_sam =[];
              %% backward random loop generator
                [rightConfig,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,true,nobst_samples,initAngle);
                rightConfig = [rightConfig, b_sam];
                % merge configure together
                numRight=size(rightConfig,2);
                if numRight > 0 
                   newConfig=leftConfig * ones(1,numRight);
                   newConfig=[newConfig;rightConfig];
                   newConfig(DOF,:)=pi * ones(1,numRight);  %% the last joint is always pi
                end
             end
             if j==DOF-3 | j==DOF-2
                % link j+2 to link DOF forming a new closed-chain
                % the baseLinkLength is also updated as the length of newVec
                newlinklength=[];
                newlinklength(1:j)=linklength(1:j);   %' * ones(1,numConfig)
                newlinklength(j+1)=baseLinkLength;
                b_sam = [];
              %% backward random loop generator
                [leftConfig,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,false,nobst_samples,initAngle);
                % merge configure together
                leftConfig = [lefConfig,b_sam];
                leftConfig = leftConfig(1:j,:);
                numLeft=size(leftConfig,2);
                rightConfig = [rightConfig;pi];
                if numLeft > 0 
                   newConfig=rightConfig * ones(1,numLeft);
                   newConfig=[leftConfig;newConfig];
                   newConfig(DOF,:)=pi * ones(1,numLeft);  %% the last joint is always pi
                end
             end
          end
          o_samples=[o_samples,newConfig];
        end
     end
  end
end

nobst_samples = 3;
rEnd = [linklength(DOF),0]';
for i=1:numDiaObst, % check every point obstacle, see if it can be touched by a link
  vert = poly_dilate.obst(i).coord;
  ext_vert = poly_dilate.obst(i).ext_vert;
  peri = poly_dilate.obst(i).peri;  %% perimeter of workspace obstacle, for estimating how long for sampling 
  num_vert = size(vert, 2);
  angle_intv = poly_dilate.obst(i).angle_interval;
  
  vertR = vert - rEnd;
  obstDist = vecnorm(vert);
  obstDistR = vecnorm(vertR);
  
  
  %% in the following we generate two end points of a link, and then forming closed loop by sampling
  %% two closed chains, one on the left, and the other on the right
  for j=0:DOF-2, % right open chain has at least two links
    for k=1:num_vert,  %iterate through all vertices of each obstacle
        pt = vert(:,k);
        rpt = vertR(:,k);
        intv = angle_intv(:,k);
        evert = ext_vert(:,k); 
        if obstDist(k) <= r_len(j+1) && obstDist(k) >= l_len(j+1) ...
           && obstDistR(k) <= r_len(DOF-j-1) && obstDistR(k) >= l_len(DOF-j-1) % meaning obstacle i can be intesected by link j+1
          %%% link 1 ... j, j+1, is equivalent to a closed chain, and the
          %%% remaining links as a second closed chain
          leftConfig=[];
          rightConfig=[];
          in_interval = false;
          newConfig = [];
          leftEnd = [0,0]';
          rightEnd = rEnd;
          if j==0
              angle= atan2(-pt(2),-pt(1));
              in_interval = CheckAngleInInterval(angle, intv);
              leftConfig = angle + pi;
              if in_interval
                leftEnd=leftEnd + pt * r_len(j+1)/obstDist(k);
                %% newVec=rightEnd-leftEnd;
              end
          else
              if j==1   %% link 2 touch the obstacle
               %% compute the intersection of the circle of l1 based at (0,0) and the circle of l2 based at pt
                 tc1 = atan2(pt(2),pt(1));
                 alpha1 = acos((linklength(1)^2 + obstDist(k)^2  - linklength(2)^2)/(2 * linklength(1) * obstDist(k)));
                 tup1 = tc1 + alpha1;
                 tdown1 = tc1 - alpha1;
                 t1 = tdown1 + rand(1) * (tup1 - tdown1);
                 dist_end1 = [linklength(1) * cos(t1), linklength(1) * sin(t1)]';
                 vec1 = dist_end1 - pt;
                 angle = atan2(vec1(2), vec1(1));
                 in_interval =  CheckAngleInInterval(angle, intv);
                 if in_interval
                     leftConfig = [t1,angle + pi]';
                     leftEnd =leftEnd + (sum([linklength(1),linklength(2)]' .* [cos(leftConfig),sin(leftConfig)]))';
                     %% newVec=rightEnd-leftEnd;
                 end
              else
                 if j == DOF -3   %% link DOF-2 touch the obstacle
                     tc1 = atan2(rpt(2),rpt(1));
                     alpha1 = acos((linklength(DOF-1)^2 + obstDistR(k)^2  - linklength(DOF-2)^2)/(2 * linklength(DOF-1) * obstDistR(k)));
                     tup1 = tc1 + alpha1;
                     tdown1 = tc1 - alpha1;
                     t1 = tdown1 + rand(1) * (tup1 - tdown1);
                     dist_end1 = [linklength(DOF-1) * cos(t1), linklength(DOF-1) * sin(t1)]';
                     vec1 = dist_end1 - rpt;
                     angle = atan2(vec1(2), vec1(1));
                     in_interval =  CheckAngleInInterval(angle, intv);
                     if in_interval
                        rightConfig = [angle, t1+pi]';
                        rightEnd =rightEnd - (sum([linklength(DOF-2),linklength(DOF-1)]' .* [cos(rightConfig),sin(rightConfig)]))';
                     end
                 else  
                     if j==DOF-2  %% link DOF-1 touch the obstacle
                        rightConfig= atan2(-rpt(2),-rpt(1));
                        in_interval = CheckAngleInInterval(rightConfig, intv);
                        if in_interval
                           rightEnd=rightEnd - linklength(DOF-1) * [cos(rightConfig), sin(rightConfig)]';;
                        end
                     else
                        [newConfig]=MidLinkRandomLoopGenerator(linklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR, vert, angle_intv, pt,j+1,k,peri, nobst_samples);  %% collision loop closure generator, not this is different from regular loop closure generator
                     end
                 end
              end
          end
          if in_interval,
             %%newVec = rightEnd - leftEnd;
             %%baseLinkLength=norm(newVec);
             %%initAngle=atan2(newVec(2),newVec(1)); 
             if j==0 | j==1
                 start_link_index = j;
                 start_vert_index = k;
                 ccw = true;
                 rightConfig1=[];
                 [rightConfig1] = TraceObstHomotopy(linklength, CollisionVarCritRadiusL,CollisionVarCritRadiusR, nobst_samples,...
                                                    vert, angle_intv,...
                                                    start_link_index, start_vert_index, leftEnd,rightEnd, peri,ccw);
                 rightConfig = [rightConfig, rightConfig1];


                 start_link_index = j;
                 start_vert_index = k;
                 ccw = false;
                 rightConfig1 = [];
                 [rightConfig1] = TraceObstHomotopy(linklength, CollisionVarCritRadiusL,CollisionVarCritRadiusR, nobst_samples,...
                                                    vert, angle_intv,...
                                                    start_link_index, start_vert_index, leftEnd,rightEnd, peri, ccw);
                 rightConfig = [rightConfig, rightConfig1];                      
                  % merge configure together
                 numRight=size(rightConfig,2);
                 if numRight > 0 
                    newConfig=leftConfig * ones(1,numRight);
                    newConfig=[newConfig;rightConfig];
                    newConfig(DOF,:)=pi * ones(1,numRight);  %% the last joint is always pi
                 end
             end
             if j==DOF-3 | j==DOF-2
                % link j+2 to link DOF forming a new closed-chain
                % the baseLinkLength is also updated as the length of newVec
                newlinklength=[];
                newlinklength(1:j)=linklength(1:j);   %' * ones(1,numConfig)
                newlinklength(j+1)=baseLinkLength;
                b_sam = [];
              %% backward random loop generator
                [leftConfig,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,true,false,nobst_samples,initAngle);
                % merge configure together
                leftConfig = [lefConfig,b_sam];
                leftConfig = leftConfig(1:j,:);
                numLeft=size(leftConfig,2);
                rightConfig = [rightConfig;pi];
                if numLeft > 0 
                   newConfig=rightConfig * ones(1,numLeft);
                   newConfig=[leftConfig;newConfig];
                   %%newConfig(DOF,:)=pi * ones(1,numLeft);  %% the last joint is always pi
                end
             end
          end
          o_samples=[o_samples,newConfig];
        end
     end
  end
end

num_fl_samples = size(samples,2);  %% samples are based on floatIndex (recordered chain)
num_bd_samples = size(b_samples, 2);  %% number of bound samples
num_o_samples = size(o_samples,2);  %% o_samples are based on original close chain
fprintf(1,'regular samples=%d, num_bd_samples=%d, num_obst_samples=%d \n', num_fl_samples, num_bd_samples, num_o_samples);
% % for i=1:num_o_samples,
% %     csample = o_samples(:,i);
% %     samples = [samples, csample(floatIndex)];
% % end
%% final step: collision checking
count1=0;
%%num_trials=size(samples,2);
out=[];
for i=1:num_fl_samples,
    cspace=samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count1=count1+1;
        out(:,count1)=ConvertNormal(cspace);
    end
end

count2=0;
bound_samples =[];
for i=1:num_bd_samples,
    cspace=b_samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count2=count2+1;
        bound_samples(:,count2)=ConvertNormal(cspace);
    end
end

count3=0;
obst_samples =[];
for i=1:num_o_samples,
    cspace=o_samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count3=count3+1;
        obst_samples(:,count3)=ConvertNormal(cspace);
    end
end
fprintf(1,'nonbound noncollision samples =%d, bound noncollision samples =%d, near cobs non-collision samples=%d \n', count1, count2, count3);
% %                 ind1 = j + 1;  %% starting iterating along increasing link indices
% %                 mod_length = 0; %%sum(linklength(1:ind1));  %% starting mod_length, recall mod_
% %                 
% %                 ind2 = k;  %% starting iterating vertics ccw
% %                 rsub_leftConfig = [];
% %                 count = 0;
% %                 while mod_length < peri
% %                     % link j+2 to link DOF forming a new closed-chain
% %                     % the baseLinkLength is also updated as the length of newVec
% %                     newlinklength=[];
% %                     newlinklength(1:DOF-ind1-1)=linklength(ind1+1:DOF-1);   %' * ones(1,numConfig)
% %                     newlinklength(DOF-ind1)=baseLinkLength;
% %                     b_sam =[];
% %                     rightConfig1= [];
% %                     
% %                     % backward random loop generator
% %                     [rightConfig1,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,true,nobst_samples,initAngle);
% %                     rightConfig1 = [rightConfig1, b_sam];
% %                     sz_r= size(rightConfig1,2);
% %                     sz_l = length(rsub_leftConfig);
% %                     if sz_r > 0
% %                       if sz_l > 0
% %                         cfg= [rsub_leftConfig * ones(1,sz_r); rightConfig1];
% %                         rightConfig = [rightConfig, cfg];
% %                       else
% %                         rightConfig = [rightConfig, rightConfig1];
% %                       end
% %                     end
% %                     
% %                     if ind1 + 1 >= DOF - 2    %% have investigated all movable links in the closed chain
% %                         break;
% %                     end
% %                     
% %                     in_interval1 = false;
% %                    
% %                     while ~in_interval1 && count < num_vert   %% can not continue enclosing the obstacle with desired angle
% %                         ind2 = mod(ind2 + 1,num_vert);
% %                         diffVec = vert(:,ind2) - leftEnd;
% %                         ang = atans2(diffVec(2), diffVec(1));
% %                         in_interval1 = CheckAngleInInterval(ang, angle_intv(:,ind2));
% %                         count = count + 1;
% %                     end
% %                     if in_interval1
% %                       dist = norm(diffVec);
% %                       mod_length = mod_length + linklength(ind1);
% %                       rsub_leftConfig = [rsub_leftConfig;ang]; 
% %                       leftEnd = leftEnd + linklength(ind1 + 1) * [cos(ang), sin(ang)]'; 
% %                       if dist > linklength(ind1 + 1)
% %                          ind2 = mod(ind2 - 1,num_vert);  
% %                          count = count - 1;
% %                       end
% %                       ind1 = ind1 + 1;
% %                       newVec = rightEnd - leftEnd;
% %                       baseLinkLength=norm(newVec);
% %                       initAngle=atan2(newVec(2),newVec(1)); 
% %                     else
% %                         break;
% %                     end
% %                 end
                
% %                 ind1 = j + 1;  %% starting iterating along increasing link indices
% %                 mod_length = 0; %%sum(linklength(1:ind1));  %% starting mod_length, recall mod_
% %                 ind2 = k;  %% starting iterating vertics cw
% %                 rsub_leftConfig = [];
% %                 count = 0;
% %                 while mod_length < peri
% %                     % link j+2 to link DOF forming a new closed-chain
% %                     % the baseLinkLength is also updated as the length of newVec
% %                     newlinklength=[];
% %                     newlinklength(1:DOF-ind1-1)=linklength(ind1+1:DOF-1);   %' * ones(1,numConfig)
% %                     newlinklength(DOF-ind1)=baseLinkLength;
% %                     b_sam =[];
% %                     rightConfig1= [];
% %                     
% %                     % backward random loop generator
% %                     [rightConfig1,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,true,nobst_samples,initAngle);
% %                     rightConfig1 = [rightConfig1, b_sam];
% %                     sz_r= size(rightConfig1,2);
% %                     sz_l = length(rsub_leftConfig);
% %                     if sz_r > 0
% %                       if sz_l > 0
% %                         cfg= [rsub_leftConfig * ones(1,sz_r); rightConfig1];
% %                         rightConfig = [rightConfig, cfg];
% %                       else
% %                         rightConfig = [rightConfig, rightConfig1];
% %                       end
% %                     end
% %                     
% %                     if ind1 + 1 >= DOF - 2    %% have investigated all movable links in the closed chain
% %                         break;
% %                     end
% %                     
% %                     in_interval1 = false;
% %                    
% %                     while ~in_interval1 && count < num_vert   %% can not continue enclosing the obstacle with desired angle
% %                         ind2 = mod(ind2 - 1,num_vert);
% %                         diffVec = vert(:,ind2) - leftEnd;
% %                         ang = atans2(diffVec(2), diffVec(1));
% %                         in_interval1 = CheckAngleInInterval(ang, angle_intv(:,ind2));
% %                         count = count + 1;
% %                     end
% %                     if in_interval1
% %                       dist = norm(diffVec);
% %                       mod_length = mod_length + linklength(ind1);
% %                       rsub_leftConfig = [rsub_leftConfig;ang]; 
% %                       leftEnd = leftEnd + linklength(ind1 + 1) * [cos(ang), sin(ang)]'; 
% %                       if dist > linklength(ind1 + 1)
% %                          ind2 = mod(ind2 + 1,num_vert);  
% %                          count = count - 1;
% %                       end
% %                       ind1 = ind1 + 1;
% %                       newVec = rightEnd - leftEnd;
% %                       baseLinkLength=norm(newVec);
% %                       initAngle=atan2(newVec(2),newVec(1)); 
% %                     else
% %                         break;
% %                     end
% %                 end