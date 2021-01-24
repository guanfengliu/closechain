function [regular_samples, bound_samples, obst_samples] = RandomSampleCompGen2(mpData)
%% function that adopt topology-component based sampling algorithm
%% @mapData includes all required input data structures, see the following assignment 


linklength = mpData.linklength; %% @linklength, the original link length vector
thickness = mpData.thickNess;
%% the following two data structures is the radii of the set of critical circles
% counting from left to right, and second is from right to let
%@CollisionVarCritRadiusL=[l1+l2, abs(l1-l2), l1+l2+l3, abs(l1+l2-l3), abs(l1-l2+l3), abs(l1-l2-l3),
%...], which are very important for determining the sampling range of each
%joint angles

CollisionVarCritRadiusL = mpData.CollisionVarCritRadiusL;
CollisionVarCritRadiusR = mpData.CollisionVarCritRadiusR;

%% triangulated obstacle for collision checking
polyObst = mpData.polyObst;

%% enveloping polygon for sampling compatible random configurations
poly_dilate = mpData.poly_dilate;

%% other input parameters
%enable_bound = mapData.enable_bound;  %% whether or not enable boundary variety sampling
%nonbsamples = mapData.nonbsamples;  %% number of regular samples
nobst_samples = mpData.nobst_samples;  %% how many compatible samples generated around each obstacle


%% first step: using random loop generator to generate regular configurations and boundary configurations
mpData.left_2_right = false;
mpData.init_angle = 0;
[samples, b_samples]=RandomLoopGeneratorGen2(mpData); 

    
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
% for i=1:numDiaObst, % check every point obstacle, see if it can be touched by a link
%   vert = poly_dilate.obst(i).coord;
%   num_vert = size(vert, 2);
%   angle_intv = poly_dilate.obst(i).angle_interval;
%   vertR = vert - rEnd;
%   obstDist = vecnorm(vert);
%   obstDistR = vecnorm(vertR);
%   %% in the following we generate two end points of a link, and then forming closed loop by sampling
%   %% two closed chains, one on the left, and the other on the right
%   for j=0:DOF-2, % right open chain has at least two links
%     for k=1:num_vert,  %iterate through all vertices of each obstacle
%         pt = vert(:,k);
%         rpt = vertR(:,k);
%         intv = angle_intv(:,k);
%         newVec = [];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
%         if obstDist(k) <= r_len(j+1) && obstDist(k) >= l_len(j+1) ...
%            && obstDistR(k) <= r_len(DOF-j-1) && obstDistR(k) >= l_len(DOF-j-1) % meaning obstacle i can be intesected by link j+1
%           %%% link 1 ... j, j+1, is equivalent to a closed chain, and the
%           %%% remaining links as a second closed chain
%           leftConfig=[];
%           rightConfig=[];
%           in_interval = false;
%           newConfig = [];
%           if j==0
%               angle= atan2(-pt(2),-pt(1));
%               in_interval = CheckAngleInInterval(angle, intv);
%               leftConfig = angle + pi;
%               if in_interval
%                 leftEnd=pt * r_len(j+1)/obstDist(k);
%                 newVec=[newVec, rEnd-leftEnd];
%               end
%           else
%               if j==1   %% link 2 touch the obstacle
%                %% compute the intersection of the circle of l1 based at (0,0) and the circle of l2 based at pt
%                  tc1 = atan2(pt(2),pt(1));
%                  alpha1 = acos((linklength(1)^2 + obstDist(k)^2  - linklength(2)^2)/(2 * linklength(1) * obstDist(k)));
%                  tup1 = tc1 + alpha1;
%                  tdown1 = tc1 - alpha1;
%                  for ddd=1:nobst_samples+1,
%                    if ddd <= nobst_samples
%                       t1 = tdown1 + rand(1) * (tup1 - tdown1);
%                    else
%                       t1 = tc1;
%                    end
%                    dist_end1 = [linklength(1) * cos(t1), linklength(1) * sin(t1)]';
%                    vec1 = dist_end1 - pt;
%                    angle = atan2(vec1(2), vec1(1));
%                    in_interval =  CheckAngleInInterval(angle, intv);
%                    if in_interval
%                       lConfig = [t1;angle + pi];
%                       leftEnd =(sum([linklength(1),linklength(2)]' .* [cos(lConfig),sin(lConfig)]))';
%                       leftConfig = [leftConfig, lConfig];
%                       newVec=[newVec, rEnd-leftEnd];
%                    end
%                  end
%               else
%                  if j == DOF -3   %% link DOF-2 touch the obstacle
%                      tc1 = atan2(rpt(2),rpt(1));
%                      alpha1 = acos((linklength(DOF-1)^2 + obstDistR(k)^2  - linklength(DOF-2)^2)/(2 * linklength(DOF-1) * obstDistR(k)));
%                      tup1 = tc1 + alpha1;
%                      tdown1 = tc1 - alpha1;
%                      for ddd=1:nobst_samples+1,
%                        if ddd <= nobst_samples
%                           t1 = tdown1 + rand(1) * (tup1 - tdown1);
%                        else
%                           t1 = tc1;
%                        end
%                        dist_end1 = [linklength(DOF-1) * cos(t1), linklength(DOF-1) * sin(t1)]';
%                        vec1 = dist_end1 - rpt;
%                        angle = atan2(vec1(2), vec1(1));
%                        in_interval =  CheckAngleInInterval(angle, intv);
%                        if in_interval
%                           rConfig = [angle;t1+pi];
%                           rightConfig = [rightConfig,rConfig];
%                           newVec =[newVec, rEnd - (sum([linklength(DOF-2),linklength(DOF-1)]' .* [cos(rConfig),sin(rConfig)]))'];
%                        end
%                      end
%                  else  
%                      if j==DOF-2  %% link DOF-1 touch the obstacle
%                         rightConfig= atan2(-rpt(2),-rpt(1));
%                         in_interval = CheckAngleInInterval(rightConfig, intv);
%                         if in_interval
%                            newVec=[newVec, rEnd - linklength(DOF-1) * [cos(rightConfig), sin(rightConfig)]'];
%                         end
%                      else
%                         [newConfig]=MidLinkRandomLoopGeneratorOldEff(linklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR, pt, intv, j+1, nobst_samples);  %% collision loop closure generator, not this is different from regular loop closure generator
%                      end
%                  end
%               end
%           end
%           
%           if size(newVec,2) > 0
%              baseLinkLength=vecnorm(newVec);
%              initAngle=atan2(newVec(2,:),newVec(1,:)); 
%              if j==0 | j==1
%                 numLeft = length(baseLinkLength);
%                 % link j+2 to link DOF forming a new closed-chain
%                 % the baseLinkLength is also updated as the length of newVec
%                 for ddd=1:numLeft,
%                   lcfg = leftConfig(:,ddd);
%                   newlinklength=[];
%                   newlinklength(1:DOF-j-2)=linklength(j+2:DOF-1);   %' * ones(1,numConfig)
%                   newlinklength(DOF-j-1)=baseLinkLength(ddd);
%                   b_sam =[];
%                  %% backward random loop generator
%                   [rightConfig,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,true,1,initAngle(ddd));
%                   rightConfig = [rightConfig, b_sam];
%                   % merge configure together
%                   numRight=size(rightConfig,2);
%                   if numRight > 0 
%                      newConfig=lcfg * ones(1,numRight);
%                      newConfig=[newConfig;rightConfig];
%                      newConfig(DOF,:)=pi * ones(1,numRight);  %% the last joint is always pi
%                   end
%                 end
%              end
%              if j==DOF-3 | j==DOF-2
%                 numRight = length(baseLinkLength);
%                 % link j+2 to link DOF forming a new closed-chain
%                 % the baseLinkLength is also updated as the length of newVec
%                 for ddd=1:numRight,
%                    rcfg = rightConfig(:,ddd);  
%                    newlinklength=[];
%                    newlinklength(1:j)=linklength(1:j);   %' * ones(1,numConfig)
%                    newlinklength(j+1)=baseLinkLength(ddd);
%                    b_sam = [];
%                   %% backward random loop generator
%                    [leftConfig,b_sam]=RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,true,false,1,initAngle(ddd));
%                    % merge configure together
%                    leftConfig = [leftConfig,b_sam];
%                    numLeft=size(leftConfig,2);
%                    if numLeft > 0 
%                       rcfg = [rcfg;pi];
%                       leftConfig = leftConfig(1:j,:);
%                       newConfig=rcfg * ones(1,numLeft);
%                       newConfig=[leftConfig;newConfig];
%                       newConfig(DOF,:)=pi * ones(1,numLeft);  %% the last joint is always pi
%                    end
%                 end
%              end
%           end
%           o_samples=[o_samples,newConfig];
%         end
%      end
%   end
% end
% rEnd = [linklength(DOF),0]';
for i=1:numDiaObst, % check every point obstacle, see if it can be touched by a link
  vert = poly_dilate.obst(i).coord;
  %ext_vert = poly_dilate.obst(i).ext_vert;
  %peri = poly_dilate.obst(i).peri;  %% perimeter of workspace obstacle, for estimating how long for sampling 
  num_vert = size(vert, 2);
  angle_intv = poly_dilate.obst(i).angle_interval;
  
  vertR = vert - rEnd;
  obstDist = vecnorm(vert);
  obstDistR = vecnorm(vertR);
  
  %% compData is the data structure for planning compatible configurations with obstacles
  compData.vert = vert;
  compData.angle_intv = angle_intv;
  %% in the following we generate two end points of a link, and then forming closed loop by sampling
  %% two closed chains, one on the left, and the other on the right
  for j=0:DOF-2, % right open chain has at least two links
    for k=1:num_vert,  %iterate through all vertices of each obstacle
        pt = vert(:,k);
        rpt = vertR(:,k);
        intv = angle_intv(:,k);
        %evert = ext_vert(:,k); 
        newVec = [];
        if obstDist(k) <= r_len(j+1) && obstDist(k) >= l_len(j+1) ...
           && obstDistR(k) <= r_len(DOF-j-1) && obstDistR(k) >= l_len(DOF-j-1) % meaning obstacle i can be intesected by link j+1
          %%% link 1 ... j, j+1, is equivalent to a closed chain, and the
          %%% remaining links as a second closed chain
          leftConfig=[];
          rightConfig=[];
          in_interval = false;
          newConfig = [];
          leftEnd = [];
          leftEnd_old = [0,0]';
          rightEnd_old = rEnd;
          rightEnd = [];
          if j==0
              angle= atan2(-pt(2),-pt(1));
              in_interval = CheckAngleInInterval(angle, intv);
              if in_interval
                leftConfig = angle + pi;  
                leftEnd=leftEnd_old + pt * r_len(j+1)/obstDist(k);
                rightEnd = rightEnd_old;
                leftEnd=leftEnd * ones(1,nobst_samples);
                rightEnd = rightEnd * ones(1,nobst_samples);
                %% newVec=rightEnd-leftEnd;
              end
          else
              if j==1   %% link 2 touch the obstacle
               %% compute the intersection of the circle of l1 based at (0,0) and the circle of l2 based at pt
                 tc1 = atan2(pt(2),pt(1));
                 alpha1 = acos((linklength(1)^2 + obstDist(k)^2  - linklength(2)^2)/(2 * linklength(1) * obstDist(k)));
                 tup1 = tc1 + alpha1;
                 tdown1 = tc1 - alpha1;
                 for ddd=1:nobst_samples+1,
                   if ddd <= nobst_samples
                      t1 = tdown1 + rand(1) * (tup1 - tdown1);
                   else
                      t1 = tc1;
                   end
                   dist_end1 = [linklength(1) * cos(t1), linklength(1) * sin(t1)]';
                   vec1 = dist_end1 - pt;
                   angle = atan2(vec1(2), vec1(1));
                   in_interval =  CheckAngleInInterval(angle, intv);
                   if in_interval
                       lConfig = [t1;angle + pi];
                       leftConfig = [leftConfig,lConfig];
                       leftEnd =[leftEnd,leftEnd_old + (sum([linklength(1),linklength(2)]' .* [cos(lConfig),sin(lConfig)]))'];
                       rightEnd = [rightEnd, rightEnd_old];
                      %% newVec=rightEnd-leftEnd;
                   end
                 end
              else
                 if j == DOF -3   %% link DOF-2 touch the obstacle
                     tc1 = atan2(rpt(2),rpt(1));
                     alpha1 = acos((linklength(DOF-1)^2 + obstDistR(k)^2  - linklength(DOF-2)^2)/(2 * linklength(DOF-1) * obstDistR(k)));
                     tup1 = tc1 + alpha1;
                     tdown1 = tc1 - alpha1;
                     for ddd=1:nobst_samples+1,
                       if ddd <= nobst_samples
                          t1 = tdown1 + rand(1) * (tup1 - tdown1);
                       else
                          t1 = tc1;
                       end
                       dist_end1 = [linklength(DOF-1) * cos(t1), linklength(DOF-1) * sin(t1)]';
                       vec1 = dist_end1 - rpt;
                       angle = atan2(vec1(2), vec1(1));
                       in_interval =  CheckAngleInInterval(angle, intv);
                       if in_interval
                          rConfig = [angle, t1+pi]';
                          rightConfig = [rightConfig,rConfig];
                          rightEnd =[rightEnd,rightEnd_old - (sum([linklength(DOF-2),linklength(DOF-1)]' .* [cos(rConfig),sin(rConfig)]))'];
                          leftEnd = [leftEnd,leftEnd_old];                      
                       end
                     end
                 else  
                     if j==DOF-2  %% link DOF-1 touch the obstacle
                        rightConfig= atan2(-rpt(2),-rpt(1));
                        in_interval = CheckAngleInInterval(rightConfig, intv);
                        if in_interval
                           rightEnd=[rightEnd, rightEnd_old - linklength(DOF-1) * [cos(rightConfig), sin(rightConfig)]'];
                           leftEnd = [leftEnd, leftEnd_old];
                           leftEnd=leftEnd * ones(1,nobst_samples);
                           rightEnd = rightEnd * ones(1,nobst_samples);
                        end
                     else
                        %%compData.pt=pt;
                        compData.start_link_index = j+1;
                        compData.start_vert_index = k;
                        [newConfig]=MidLinkRandomLoopGeneratorEffGen2(mpData,compData);  %% collision loop closure generator, not this is different from regular loop closure generator
                     end
                 end
              end
          end
          noSamp = size(leftEnd,2);
          if noSamp > 0
             if j==0 | j==1
                 compData.start_link_index = j;
                 compData.start_vert_index = k;

                 for ddd=1:noSamp,
                   rightConfig1=[];
                   compData.leftEnd = leftEnd(:,ddd);
                   compData.rightEnd = rightEnd(:,ddd);
                   compData.ccw = true;
                   compData.nobst_samples = 1;
                   [rightConfig1] = TraceObstHomotopyGen2(mpData,compData);
                   rightConfig = [rightConfig, rightConfig1];
                   compData.ccw = false;
                   rightConfig1 = [];
                   [rightConfig1] = TraceObstHomotopyGen2(mpData,compData);
                   rightConfig = [rightConfig, rightConfig1];                      
                   % merge configure together
                   numRight=size(rightConfig,2);
                   lcfg = leftConfig(:,noSamp);
                   if numRight > 0 
                      ncfg=lcfg * ones(1,numRight);
                      ncfg=[ncfg;rightConfig];
                      ncfg(DOF,:)=pi * ones(1,numRight);  %% the last joint is always pi
                      newConfig = [newConfig,ncfg];
                   end
                 end
             end
             if j==DOF-3 | j==DOF-2
                newVec = rightEnd - leftEnd;
                baseLinkLength=vecnorm(newVec);
                initAngle=atan2(newVec(2,:),newVec(1,:));  
                for ddd=1:noSamp,
                  % link j+2 to link DOF forming a new closed-chain
                  % the baseLinkLength is also updated as the length of newVec
                  newlinklength=[];
                  newlinklength(1:j)=linklength(1:j);   %' * ones(1,numConfig)
                  newlinklength(j+1)=baseLinkLength(ddd);
                  b_sam = [];
                 %% backward random loop generator
                  mpData.enable_bound = true;
                  mpData.left_2_right = false;
                  mpData.init_angle = initAngle(ddd);
                  mpData.nonbsamples = 1;
                  [leftConfig,b_sam]=RandomLoopGeneratorGen2(mpData);
                  % merge configure together
                  leftConfig = [leftConfig,b_sam];
                  numLeft=size(leftConfig,2);
                  rcfg = rightConfig(:,ddd);
                  if numLeft > 0 
                     leftConfig = leftConfig(1:j,:);
                     rcfg = [rcfg;pi];
                     ncfg=rcfg * ones(1,numLeft);
                     ncfg=[leftConfig;ncfg];
                     newConfig = [ newConfig, ncfg];
                   %%newConfig(DOF,:)=pi * ones(1,numLeft);  %% the last joint is always pi
                  end
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
regular_samples=[];
for i=1:num_fl_samples,
    cspace=samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count1=count1+1;
        regular_samples(:,count1)=ConvertNormal(cspace);
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