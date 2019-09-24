%% example 3,  Author: Leon G.F. Liu  09/23/2019
close all, clear all
%% linklengths
linklength=[0.5512,1.9457,1.2131,2.9482,4.5684,0.3,0.3, 5.7815];
%% 6 point obstacles
pos1=[4,2]';
pos2=[3,1]';
pos3=[2,-3]';
pos4=[3,-2]';
pos5=[0.5,-2]';
pos6=[4,-1]';

obst=[pos1,pos2,pos3,pos4,pos5,pos6]; %form a matrix, each column being a point

%% start and goal configs
start_config=[-0.6363+0.0349 -1.2183+0.0349 0.0416+0.0349 1.9416+0.0349 -0.1416+0.0349  -0.3255  -2.7811 pi];
end_config=[-0.9063-0.0349 0.8648-0.0349 0.0416-0.0349 -2.0416-0.0349 0.3416-0.0349  0.3255  2.7811 pi];
%% other parameters
thickNess=0.02;
nsamples=30;
num_internal_loops=5;
neighbors=10;

%% call main motion plan function of closed chains
ClosedChainMotionPlan(linklength,obst,start_config,end_config,thickNess,nsamples,num_internal_loops,neighbors);