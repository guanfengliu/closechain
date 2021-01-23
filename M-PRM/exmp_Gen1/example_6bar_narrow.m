%% 12-bar narrow passage example ,  Author: Leon G.F. Liu  09/23/2019
%% example 2,  Author: Leon G.F. Liu  09/23/2019
close all, clear all
%% link lengths
linklength=[3.2000, 2.0000, 2.5457, 4.5684, 5, 7.5815];
pos1=[4,1.9]'; %%[4,-6]'; %%[4,-2.8]';
pos2=[4,2.5]';

%% set up the obstacles, to do, use a function to create obstacles
obst=[];
obst.numObst = 2;
obst.obstacle(1).coord = pos1;
obst.obstacle(2).coord = pos2;

numApp=8; %% number of approximating point obtacle for each original point obstacle
angles=-pi:2*pi/numApp:pi;
angles=angles(1:numApp);
normal_vec = [cos(angles);sin(angles)]; 
obst.obstacle(1).normal_vec = normal_vec;
obst.obstacle(2).normal_vec = normal_vec;
num_links = length(linklength);

%% dilate factor
dilate_epsilon = 0.1;
%% First: compute the descending links (reordered link length vector) and their radii of critical circles (from left and right)
%% Second: preprocessing the obstacles to calculate the incident-link cone which does not lead to collision with obstacles
[BoundVarCritRadiusL,BoundVarCritRadiusR,poly_dilate,polyObst] = preprocessCloseChainGen(linklength,obst, dilate_epsilon);

%% link thickness for collision checking
thickNess=0.02; 

j_start=2;
j_end=3;
pos_left = (pos1 + pos2)/2.0;
pos_right = (pos1 + pos2)/2.0;
numPair = 2;
%% the link index which as to pass through narrow passages
%[start_config,end_config]=GenerateValidStartEndNarrowEff(linklength,thickNess,BoundVarCritRadiusL,BoundVarCritRadiusR,polyObst, obst, pos_left,pos_right,j_start,j_end, numPair);
startc(:,1) = [0.8876, -0.1414, 0.8103, 0.0303, -2.1507, 3.1416]';
startc(:,2) = startc(:,1); %[0.8876, -0.1414, -1.2786, 1.4988, -1.0427, 3.1416]';
endc(:,1) = [0.6889, -1.4812, 1.0114, 0.6597, -1.5765, 3.1416]';
endc(:,2) = [1.5951, 0.2065, -0.5866, 0.6597, -1.5765, 3.1416]';

crit_circ = BoundVarCritRadiusL(num_links-2).linklength;
if linklength(num_links) > max(crit_circ) | linklength(num_links) < min(crit_circ)
    fprintf(1,'C-space is empty');
    return;
end

%% create roadmap for C-space
nobst_samples=10; %%10;  %% c-obst samples in each iteration
enable_bound=true;
if enable_bound
    strBound = 'wbound';
else
    strBound = 'wobound';
end
%% num_internal_loops= 50; %%100; %%30 %%50; %%50; %% 20; %% 100;  %% more number is potential to generate more c-obst samples, totally num_internal_loops * nsamples for each C-obst
neighbors= 10;  %%10;
nonbsamples= 100; %% 200;  %%500;%% non boundary samples

%% intialize roadmap
roadmap.samples = [];
roadmap.edges = [];
roadmap.edge_lengths = [];
roadmap.nsamples=0;
[roadmap,bound_samples, obst_samples,sampleTime,mapTime] = PRM_REV (roadmap, @()(RandomSampleClosedChainPolyEff(linklength,thickNess,BoundVarCritRadiusL,...
                                       BoundVarCritRadiusR,polyObst,poly_dilate,enable_bound,nonbsamples,nobst_samples)), ...
                                       @DistClosedChain, ...
                                       @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,linklength,thickNess)), neighbors);
totalTime = sampleTime + mapTime;
fileName1=['test',num2str(num_links),'bar',num2str(obst.numObst),'obst', num2str(nonbsamples), 'pureRanSamples',num2str(nobst_samples), 'obstSamples',strBound, '.txt'];
save(fileName1,'totalTime','-ascii');
SS=roadmap.edges(:,1)';
TT=roadmap.edges(:,2)';
weights=roadmap.edge_lengths';
G = graph(SS,TT,weights);
numComp = max(conncomp(G));
fprintf(1,'totalTime=%f, numCom=%d \n', totalTime,numComp);



%% start_final = start_config(:,i)(floatIndex);
%% end_final = end_config(:,i)(floatIndex);
%%totalTrials = size(startc,2);
totalTrials = numPair;
succStartConfig=[];
failStartConfig=[];
succEndConfig=[];
failEndConfig=[];
numFail=0;
numSucc=0;
compVec=[numComp];
fig_id=0;
for i=1:totalTrials
  fig_id=fig_id + 1;
  start_config = startc(:,i);
  fig_hnd=figure(fig_id);
  %%%start_config=RandomSampleClosedChainSingle(linklength,thickNess,BoundVarCritRadiusL,polyObst,1);
  %start_config=start_config;
  %%%end_config=RandomSampleClosedChainSingle(linklength,thickNess,BoundVarCritRadiusL,polyObst,0);
%end_config=end_config';
  %[start_config,end_config]=GenerateValidStartEnd(linklength,thickNess,CollisionVarCritRadiusL,polyObst,obst, dilatePtObst,fig_hnd,i);
  end_config = endc(:,i);
  %% draw start and goal config pair along with obstacles.
  drawStartGoalConfigPoly(linklength, start_config, end_config, obst,fig_hnd,fig_id, dilate_epsilon);
  % Add start, end nodes, with reordered link length set, floatlinks, 
  roadmap2 = AddNode2PRM_REV (start_config, roadmap, @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,linklength,thickNess)), neighbors);
  roadmap2 = AddNode2PRM_REV (end_config, roadmap2, @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,linklength,thickNess)), neighbors);
  SS=roadmap2.edges(:,1)';
  TT=roadmap2.edges(:,2)';
  weights=roadmap2.edge_lengths';
  G = graph(SS,TT,weights);
  numComp = max(conncomp(G));
  compVec=[compVec,numComp];
  route=[];
  if roadmap2.nsamples ~= roadmap.nsamples+2
     fprintf (1, 'No. %d start or goal can not be connected to roadmap, planning fails \n', i);
     failStartConfig=[failStartConfig,start_config];
     failEndConfig=[failEndConfig,end_config];
     numFail=numFail+1;
  else
    route = shortestpath(G,roadmap.nsamples+1, roadmap.nsamples+2);
    if length(route)==0
        fprintf(1,'No. %d start and goal are not connected, planning fails \n', fig_id);
        failStartConfig=[failStartConfig,start_config];
        failEndConfig=[failEndConfig,end_config];
        numFail=numFail+1;
    else
        fprintf(1,'No. %d start and goal, plan success \n', fig_id);  
        succStartConfig=[succStartConfig,start_config];
        succEndConfig=[succEndConfig,end_config];
        numSucc=numSucc+1;
    end
  end
  %% the file name to recording final path between start and goal configurations
  pathfileName=['test',num2str(num_links),'bar',num2str(obst.numObst),'obst',num2str(fig_id),'100sampleswithowithb'];
  success=drawAndSavePath(BoundVarCritRadiusL,BoundVarCritRadiusR,roadmap2, route, obst, poly_dilate, polyObst,linklength,thickNess,pathfileName,fig_id,start_config,end_config,bound_samples, obst_samples, dilate_epsilon);
end
fprintf(1,'No. of success=%d, No. of failure=%d  \n', numSucc, numFail); 
save('succ_start_config5bar2obst100samples.txt','succStartConfig','-ascii');
save('succ_end_config5bar2obst100samples.txt','succEndConfig','-ascii');
save('fail_start_config5bar2obst100samples.txt','failStartConfig','-ascii');
save('fail_end_config5bar2obst100samples.txt','failEndConfig','-ascii');


