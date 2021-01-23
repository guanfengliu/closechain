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

dilate_epsilon = 0.01;
%% compute the descending links (reordered link length vector) and their radii of critical circles (from left and right)
[BoundVarCritRadiusL,BoundVarCritRadiusR,poly_dilate,polyObst] = preprocessCloseChainGen(linklength,obst, dilate_epsilon);
%% compute the radii of critical circles (from left and right) of the original link lengths
%[o_BoundVarCritRadiusL,o_BoundVarCritRadiusR,ploy_dilate,polyObst] = preprocessCloseChainGen(linklength,obst, dilate_epsilon);
%% generate a given number of start goal config pairs, using manual or auto
%% startc and endc are w.r.t. the changed-order chain
thickNess=0.02; %0.005;

%% the link index which as to pass through narrow passages
numPair = 1;
% % j=5;  
% % pos_left = [4-linklength(j), 2.2]';
% % pos_right =[4, 1.016]';
% % start_config1 = [];
% % end_config1 = [];
% % [start_config1,end_config1]=GenerateValidStartEndNarrow(floatLinks,floatIndex, fixIndex,thickNess,BoundVarCritRadiusL,BoundVarCritRadiusR,polyObst,obst, dilatePtObst, pos_left,pos_right,j, numPair);
% % j=5;
% % pos_right = [4+linklength(j), 2.2]';
% % pos_left =[4, 2.2]';
% % start_config2 = [];
% % end_config2 = [];
% % [start_config2,end_config2]=GenerateValidStartEndNarrow(floatLinks,floatIndex, fixIndex,thickNess,BoundVarCritRadiusL,BoundVarCritRadiusR,polyObst,obst,dilatePtObst, pos_left,pos_right,j, numPair);
%%startc=[-0.6374,1.3540,-1.4597,1.1596,3.3700,0,-1.2027,1.5717,-0.4506,0.1623,2.6950,3.1416]';
%%startc =[0.1880, 0.5012 , 0.4735,0.4032, 0, -2.1958, -1.1391, 0.1973, -0.7752, 0.4264, 1.8389, 3.1416]'; 

%% test narrow-passage example 1
%startc=[-0.3800, 0.5708, 2.2956,0.6335, 0, -0.3809, -1.2904, 3.9113, 2.3753, 1.1734, 3.6964, 3.1416]';
%endc=[0.2388,1.7032,1.4992,-0.3222,0,-1.0564,-0.5959,1.1011,0.6490,1.6947,4.5497,3.1416]';

%% test narrow-passage example 2, values
% % startc(:,1) = [0.9675, -0.0236, 2.4099, 0.4755, 0, -2.8287, -0.4714, 3.4236, 1.7357, 0.1414, 3.1952, 3.1416]';
% % startc(:,2) = [0.2903, -0.2331, 1.3529, 1.1555, 0, 0.5445, 0.7097, -2.6204, -0.9393, -2.2542, -1.3210, 3.1416]'; 
% % startc(:,3)= [0.6713, -0.1164, -0.3322, 1.2827, 0, 1.0375, -0.2033, -1.7436, -4.8632, -2.6913, -0.7116, 3.1416]';
% % endc(:,1) =[3.0833, 1.0082, 3.9999, 0.5321, 0.8551, 1.6421, -1.0611, 0.0881, -0.1670, -1.0242, 0.9857, 3.1416]';
% % endc(:,2) = [2.9817, 1.6028, 2.2089, 1.6319, 3.1186, -0.0443, -0.5205, -2.5485, 0.2425, -0.9420, 0.8815, 3.1416]';
% % endc(:,3) = [1.8037, 1.3383, 1.5009, 2.2387, 3.4863, -1.0145, -0.0170, -0.5152, -1.4631, -0.9107, 1.0629, 3.1416]';
% % endc(:,4) = [1.4713, 1.3519, 1.2631, 1.1003, -0.4897, 3.4070, -0.4198, 0.4892, 0.9590, -1.0715, 0.8002, 3.1416]';

%% test narrow-passage example 3, values
%%startc=[-0.3800,0.5708,2.2956,0.6335,0,-0.3809,-1.2904,3.9113,2.3753,1.1734,3.6964,3.1416]';
%%endc = [1.0352,0.3137,2.1130,0.7029,-1.3335,-0.6641,-0.7527,1.8680,5.0194,1.5667,4.2604,3.1416]';
%% test narrow-passage example 2: generate start and goal
pos_left = (pos1 + pos2)/2.0;
pos_right = pos_left;
numPair=50;
[startc,endc]=GenerateValidStartEndNarrow(linklength,thickNess,BoundVarCritRadiusL,BoundVarCritRadiusR,polyObst, obst, pos_left,pos_right,4,5, numPair);

crit_circ = BoundVarCritRadiusL(num_links-2).linklength;
if linklength(num_links) > max(crit_circ) | linklength(num_links) < min(crit_circ)
    fprintf(1,'C-space is empty');
    return;
end

%% create roadmap for C-space
nobst_samples=1; %10; %%10;  %% c-obst samples in each iteration
enable_bound=true;
if enable_bound
    strBound = 'wbound';
else
    strBound = 'wobound';
end
%% num_internal_loops= 50; %%100; %%30 %%50; %%50; %% 20; %% 100;  %% more number is potential to generate more c-obst samples, totally num_internal_loops * nsamples for each C-obst
neighbors= 10;  %%10;
nonbsamples= 200; %% 200;  %%500;%% non boundary samples

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
  %%start_config = startc(:,i);
  fig_hnd=figure(fig_id);
  start_config = startc(:,i);
  end_config = endc(:,i);
  %%%start_config=RandomSampleClosedChainSingle(linklength,thickNess,BoundVarCritRadiusL,polyObst,1);
  %start_config=start_config';
  %%%end_config=RandomSampleClosedChainSingle(linklength,thickNess,BoundVarCritRadiusL,polyObst,0);
%end_config=end_config';
  %[start_config,end_config]=GenerateValidStartEnd(linklength,thickNess,CollisionVarCritRadiusL,polyObst,obst, dilatePtObst,fig_hnd,i);
  %%end_config = endc(:,j);
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


