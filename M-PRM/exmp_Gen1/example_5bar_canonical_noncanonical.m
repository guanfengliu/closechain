%% example 1,  Author: Leon G.F. Liu  09/23/2019

close all, clear all
linklength = [1, 1.3, 4, 4, 5];
%%linklength = [1, 1, 4, 4, 5];
%%linklength = [2, 2, 2, 2, 5];  %% example 2
num_links = length(linklength);
% % [floatLinks, floatIndex] = sort(linklength(1:num_links-1),'descend');
% % [fx,fixIndex] = sort(floatIndex);
% % floatLinks = [floatLinks, linklength(num_links)];
% % floatIndex = [floatIndex, num_links];
% % fixIndex = [fixIndex, num_links];
floatLinks = linklength;
floatIndex = 1:1:num_links;
fixIndex=floatIndex;
% 2 point obstacles, narrow passage exmaple
% % pos1=[1, 1]';
% % pos2=[1, 1.5]';

% 2 point obstacles, more narrow passage exmaple
% % pos1=[1, 1.1]';
% % pos2=[1, 1.4]';

%% bound example
pos3=[2, 1]';
pos4=[2,1.5]';

%%pos5=[1, -1]';
%%pos6=[1, -1.5]';
%%pos7=[2, -1]';
%%pos8=[2,-1.5]';

obst=[pos3,pos4]; %form a matrix, each column being a point
numObst = size(obst,2);
%% compute the descending links (reordered link length vector) and their radii of critical circles (from left and right)
[BoundVarCritRadiusL,BoundVarCritRadiusR,dilatePtObst,polyObst] = preprocessCloseChain(floatLinks,obst);
%% compute the radii of critical circles (from left and right) of the original link lengths
[o_BoundVarCritRadiusL,o_BoundVarCritRadiusR,dilatePtObst,polyObst] = preprocessCloseChain(linklength,obst);
crit_circ = o_BoundVarCritRadiusL(num_links-2).linklength;

if linklength(num_links) > max(crit_circ) | linklength(num_links) < min(crit_circ)
    fprintf(1,'C-space is empty');
    return;
end
%%print2dCspaceAndBound(linklength,obst,BoundVarCritRadiusL,BoundVarCritRadiusR,o_BoundVarCritRadiusL, o_BoundVarCritRadiusR,floatIndex, fixIndex);
%% create roadmap for C-space
thickNess=0.01;
nsamples=0;  %% c-obst samples in each iteration
enable_bound=1;
num_internal_loops= 0; %%30 %%50; %%50; %% 20; %% 100;  %% more number is potential to generate more c-obst samples, totally num_internal_loops * nsamples for each C-obst
neighbors=5;
nonbsamples=100;%% non boundary samples

%% intialize roadmap
roadmap.samples = [];
roadmap.edges = [];
roadmap.edge_lengths = [];
roadmap.nsamples=0;
[roadmap,bound_samples, obst_samples,sampleTime,mapTime] = PRM_REV (roadmap, @()(RandomSampleClosedChain(floatLinks,floatIndex,fixIndex,thickNess,BoundVarCritRadiusL,o_BoundVarCritRadiusL,...
                                       o_BoundVarCritRadiusR,polyObst,dilatePtObst,enable_bound,nonbsamples,...
                                       nsamples,num_internal_loops)), ...
                                       @DistClosedChain, ...
                                       @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,floatLinks,fixIndex,thickNess)), neighbors);
totalTime = sampleTime + mapTime;
fileName1=['test',num2str(num_links),'bar',num2str(numObst),'obst','computationTime100sampleswithowithb','.txt'];
save(fileName1,'totalTime','-ascii');
SS=roadmap.edges(:,1)';
TT=roadmap.edges(:,2)';
weights=roadmap.edge_lengths';
G = graph(SS,TT,weights);
numComp = max(conncomp(G));
fprintf(1,'totalTime=%f, numCom=%d \n', totalTime,numComp);

%% generate a given number of start goal config pairs, using manual or auto
%% startc and endc are w.r.t. the changed-order chain
manual = true;
[startc,endc]=GenerateValidStartEnd(floatLinks,floatIndex, fixIndex, thickNess,BoundVarCritRadiusL,polyObst,manual);

%% start_final = start_config(:,i)(floatIndex);
%% end_final = end_config(:,i)(floatIndex);
totalTrials = size(startc,2);
succStartConfig=[];
failStartConfig=[];
succEndConfig=[];
failEndConfig=[];
numFail=0;
numSucc=0;
compVec=[numComp];
for i=1:1, %totalTrials
  fig_hnd=figure(i);
  start_config = startc(:,i);
  end_config = endc(:,i);
  %% draw start and goal config pair along with obstacles.
  drawStartGoalConfig(linklength, start_config(fixIndex), end_config(fixIndex), obst, dilatePtObst,fig_hnd,i);
  % Add start, end nodes, with reordered link length set, floatlinks, 
  roadmap2 = AddNode2PRM_REV (start_config, roadmap, @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,floatLinks,fixIndex,thickNess)), neighbors);
  roadmap2 = AddNode2PRM_REV (end_config, roadmap2, @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,floatLinks,fixIndex,thickNess)), neighbors);
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
        fprintf(1,'No. %d start and goal are not connected, planning fails \n', i);
        failStartConfig=[failStartConfig,start_config];
        failEndConfig=[failEndConfig,end_config];
        numFail=numFail+1;
    else
        fprintf(1,'No. %d start and goal, plan success \n', i);  
        succStartConfig=[succStartConfig,start_config];
        succEndConfig=[succEndConfig,end_config];
        numSucc=numSucc+1;
    end
  end
  %% the file name to recording final path between start and goal configurations
  pathfileName=['test',num2str(num_links),'bar',num2str(numObst),'obst',num2str(i),'100sampleswithowithb'];
  success=drawAndSavePath(o_BoundVarCritRadiusL,o_BoundVarCritRadiusR,BoundVarCritRadiusL,BoundVarCritRadiusR,roadmap2, route, obst, dilatePtObst, polyObst,floatLinks,floatIndex,fixIndex, thickNess,pathfileName,i,start_config,end_config,bound_samples, obst_samples);
end
fprintf(1,'No. of success=%d, No. of failure=%d  \n', numSucc, numFail); 
save('succ_start_config5bar2obst100samples.txt','succStartConfig','-ascii');
save('succ_end_config5bar2obst100samples.txt','succEndConfig','-ascii');
save('fail_start_config5bar2obst100samples.txt','failStartConfig','-ascii');
save('fail_end_config5bar2obst100samples.txt','failEndConfig','-ascii');


