%% example 1,  Author: Leon G.F. Liu  09/23/2019

close all, clear all

linklength=[0.5512,1.9457,1.2131,2.9482,4.5684,5.7815];
 
% 2 point obstacles
pos1=[4,2]';
pos2=[3,1]';

obst=[pos1,pos2]; %form a matrix, each column being a point

%start and goal configs
start_config=[-0.6363 -1.2183 0.0416 1.9416 -0.1416 pi];
end_config=[-0.9063 0.8648 0.0416 -2.0416 0.3416 pi];

thickNess=0.02;
nsamples=30;
num_internal_loops=5;
neighbors=10;

%% call main motion plan function of closed chains
ClosedChainMotionPlan(linklength,obst,start_config,end_config,thickNess,nsamples,num_internal_loops,neighbors);