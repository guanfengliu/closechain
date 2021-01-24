function [cf_regular_samples, cf_bound_samples, cf_obst_samples] = RandomSampleComp(mpData)
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
nobst_samples = mpData.nobst_samples;  %% how many narrow samples generated around each obstacle


%% first step: using random loop generator to generate regular configurations and boundary configurations
mpData.left_2_right = false;
mpData.init_angle = 0;
regular_samples = [];
bound_samples = [];
[regular_samples, bound_samples]=RandomLoopGeneratorGen2(mpData); 

    
%% second step: sampling points on the collision variety
% always start from link 3, till link DoF-3, then randomly choose an
% obstacle, and randomly choose a vertex, check if it is compatible

DOF=size(linklength,2);
rEnd = [linklength(DOF),0]';
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
%% fill in the critical radii structure
mpData.l_len = l_len;
mpData.r_len = r_len;
mpData.l_len_r = l_len_r;
mpData.r_len_r = r_len_r;
numDiaObst= poly_dilate.numObst;   %%size(dilatePtObst,2);
compData.numObst = numDiaObst;
for i=1:numDiaObst, % check every point obstacle, see if it can be touched by a link
  % fill in comp Data structure
  compData.obst(i).vert = poly_dilate.obst(i).coord;
  compData.obst(i).vertR = compData.obst(i).vert - rEnd;
  compData.obst(i).obstDist = vecnorm(compData.obst(i).vert);
  compData.obst(i).obstDistR = vecnorm(compData.obst(i).vertR);
  compData.obst(i).angle_intv = poly_dilate.obst(i).angle_interval;
end  

%% obst_samples are narrow samples
obst_samples = []; %%

numSamples = 0;
count = 0;
while numSamples < nobst_samples && count < nobst_samples
    sample = [];
    % GenerateCompSampleGen3 can be replaced by topological sampling
    % algorithm function  GenerateTopoSample(mpData, compData)
    [sample] = GenerateNarrowSample(mpData,compData);
    n_sample = size(sample,2);
    if n_sample > 0
        obst_samples = [obst_samples, sample];
        numSamples = numSamples + n_sample;
    end
    count=count+1;
end

num_rg_samples = size(regular_samples,2);  %% samples are based on floatIndex (recordered chain)
num_bd_samples = size(bound_samples, 2);  %% number of bound samples
num_ob_samples = size(obst_samples,2);  %% o_samples are based on original close chain
fprintf(1,'regular samples=%d, num_bd_samples=%d, num_obst_samples=%d \n', num_rg_samples, num_bd_samples, num_ob_samples);

%% final step: collision checking
count1=0;
%%num_trials=size(samples,2);
cf_regular_samples=[];
for i=1:num_rg_samples,
    cspace=regular_samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count1=count1+1;
        cf_regular_samples(:,count1)=ConvertNormal(cspace);
    end
end

count2=0;
cf_bound_samples =[];
for i=1:num_bd_samples,
    cspace=bound_samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count2=count2+1;
        cf_bound_samples(:,count2)=ConvertNormal(cspace);
    end
end

count3=0;
cf_obst_samples =[];
for i=1:num_ob_samples,
    cspace=obst_samples(:,i);
    %% collision checking is w.r.t. the original chain
    fv = closedchainthick (linklength(1:DOF-1),cspace(1:DOF-1),thickness);
    if (~CollisionCheck(fv, polyObst))
        count3=count3+1;
        cf_obst_samples(:,count3)=ConvertNormal(cspace);
    end
end
fprintf(1,'collision free nonbound noncollision samples =%d, bound noncollision samples =%d, near cobs non-collision samples=%d \n', count1, count2, count3);