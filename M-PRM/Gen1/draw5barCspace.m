%% example 1,  Author: Leon G.F. Liu  09/23/2019

close all, clear all
%%linklength = [1, 1, 4, 4, 5];
%%linklength=[1, 1.3, 4, 4, 5];  %% example 2
linklength=[4, 4, 1.3, 1, 5];   %% example 3
%%linklength=[2, 2.3, 2.35, 3.15, 5];  %%2-long links
%%linklength=[2, 2.3, 2.6, 2.9, 3];  %% 0-long links
% 2 point obstacles
pos1=[5.5, 1]';
pos2=[2, 2]';
%%pos2 =[1,1]';

obst=[pos1,pos2]; %form a matrix, each column being a point
[BoundVarCritRadiusL,BoundVarCritRadiusR,dilatePtObst,polyObst] = preprocessCloseChain(linklength,obst);
num_links = length(linklength);
if num_links==5
   print2dCspaceAndBound(linklength,obst,BoundVarCritRadiusL,BoundVarCritRadiusR, BoundVarCritRadiusL,BoundVarCritRadiusR);   %%,roadmap,phi_traj);
end
return
