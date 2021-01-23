%% 12-bar narrow passage example ,  Author: Leon G.F. Liu  09/23/2019
%% example 2,  Author: Leon G.F. Liu  09/23/2019
close all, clear all
%% link lengths
linklength=[1.2000, 2.0000, 0.5512, 1.9457, 1.2131, 2.9482, 4.5684, 0.3000, 0.3000, 5, 2.5130,  8.5815];
% 6 point obstacles

% % pos2=[7,-6]';   %%[3,1]';
% % pos3=[2,-6]';  %%[2,-3]';
% % pos4=[8,6]'; %[3,-2]';
% % pos5=[0.5,-2]';

pos1=[4,1.9]'; %%[4,-6]'; %%[4,-2.8]';
pos2=[4,2.8]';
%%pos3=[3,2.4]';
%%pos4=[3,2]'; %%[4,2]';

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

%% data structure mpData
mpData.linklength = linklength;
mpData.obst = obst;
mpData.dilate_epsilon = 0.03; %% for collision checking
mpData.dilate_proximate_poly = 0.24; %% for generating compatible configurations
mpData.thickNess = 0.02;  % link thickness

%% compute the descending links (reordered link length vector) and their radii of critical circles (from left and right)
[mpData] = preprocessCloseChainGen2(mpData);



%% the link index which as to pass through narrow passages
pos_left = (pos1 + pos2)/2.0;
pos_right = pos_left;
numPair=50;
[startc,endc]=GenerateValidStartEndNarrowGen2(mpData, pos_left,pos_right,4,5, numPair);

crit_circ = mpData.CollisionVarCritRadiusL(num_links-2).linklength;
if mpData.linklength(num_links) > max(crit_circ) | mpData.linklength(num_links) < min(crit_circ)
    fprintf(1,'C-space is empty');
    return;
end

%% create roadmap for C-space
mpData.nobst_samples=0; %% how many random samples for each compatible fragment for each obstacle
mpData.enable_bound=true;
if mpData.enable_bound
    mpData.strBound = 'wbound';
else
    mpData.strBound = 'wobound';
end
%% num_internal_loops= 50; %%100; %%30 %%50; %%50; %% 20; %% 100;  %% more number is potential to generate more c-obst samples, totally num_internal_loops * nsamples for each C-obst
mpData.neighbors= 10;  %% how many neighbors try to link one vertex to
mpData.nonbsamples= 200; %% how many regular non boundary samples

%% intialize roadmap
roadmap.samples = [];
roadmap.edges = [];
roadmap.edge_lengths = [];
roadmap.nsamples=0;
outMpData.roadmap=roadmap;
outMpData.regular_samples=[];
outMpData.bound_samples=[];
outMpData.obst_samples=[];
outMpData.sampleTime =0;
outMpData.mapTime = 0;

[outMpData] = PRM_REV_Gen2 (outMpData, @()(RandomSampleCompGen2(mpData)), ...
                            @(x,y)(LocalPlannerClosedChainSimpleGen2(x,y,mpData)), mpData.neighbors);
totalTime = outMpData.sampleTime + outMpData.mapTime;
fileName1=['test',num2str(num_links),'bar',num2str(obst.numObst),'obst', num2str(mpData.nonbsamples), 'pureRanSamples',num2str(mpData.nobst_samples), 'obstSamples',mpData.strBound, '.txt'];
save(fileName1,'totalTime','-ascii');
roadmap = outMpData.roadmap;
SS=roadmap.edges(:,1)';
TT=roadmap.edges(:,2)';
weights=roadmap.edge_lengths';
G = graph(SS,TT,weights);
numComp = max(conncomp(G));
fprintf(1,'totalTime=%f, numCom=%d \n', totalTime,numComp);

%% use the roadmap to test the connectivity between a given number of pairs of start/goal pair
succStartConfig=[];
failStartConfig=[];
succEndConfig=[];
failEndConfig=[];
numFail=0;
numSucc=0;
fig_id=0;
for i=1:numPair
  fig_id=fig_id + 1;
  fig_hnd=figure(fig_id);
  start_config = startc(:,i);
  end_config = endc(:,i);
  %% draw start and goal config pair along with obstacles.
  drawStartGoalConfigPolyGen2(mpData,start_config, end_config,fig_hnd,fig_id);
  % Add start, end nodes, with reordered link length set, floatlinks, 
  roadmap2 = [];
  roadmap2 = AddNode2PRM_REV_Gen2(start_config,outMpData.kdtree, roadmap, @(x,y)(LocalPlannerClosedChainSimpleGen2(x,y,mpData)), mpData.neighbors);
  roadmap2 = AddNode2PRM_REV_Gen2(end_config,outMpData.kdtree, roadmap2, @(x,y)(LocalPlannerClosedChainSimpleGen2(x,y,mpData)), mpData.neighbors);
  SS=roadmap2.edges(:,1)';
  TT=roadmap2.edges(:,2)';
  weights=roadmap2.edge_lengths';
  G = graph(SS,TT,weights);
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
  mpData.bound_samples = outMpData.bound_samples;
  mpData.obst_samples = outMpData.obst_samples;
  success=drawAndSavePathGen2(mpData,roadmap2, route, pathfileName,fig_id,start_config,end_config);
end
fprintf(1,'No. of success=%d, No. of failure=%d  \n', numSucc, numFail); 
save('succ_start_config5bar2obst100samples.txt','succStartConfig','-ascii');
save('succ_end_config5bar2obst100samples.txt','succEndConfig','-ascii');
save('fail_start_config5bar2obst100samples.txt','failStartConfig','-ascii');
save('fail_end_config5bar2obst100samples.txt','failEndConfig','-ascii');


