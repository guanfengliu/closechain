function [leftConfig, endPt]=SlideLoopGenerator(linklength,CollisionVarCritRadiusL,pt,nsamples)
%% this function is use to generate configuration of a special close-chain linklengh(1) ... linklength(DOF-1), 
%% the last link linklength(DOF) sliding along a point pt, which means that joint DOF is prismatic
%% Author: Leon G.F. Liu  09/23/2019

%% first, new base linklength
baseLinkLength=(pt(1)^2+pt(2)^2)^0.5;
initAngle=atan2(pt(2),pt(1));
sz=size(linklength,2);

% the set of feasible configurations for link(DOF) intersect pt, is
% equivalent to C-space of a new closed-chain with the following
% linklengths
newLinkLength=[linklength(1:sz-1),0.5 * linklength(sz), 0.5 * linklength(sz), baseLinkLength];

%% calculate the radius of the set of critical circles w.r.t. the left anker point [0,0]
CollisionVarCritRadii=CollisionVarCritRadiusL;
if sz > 2
    critRadii=CollisionVarCritRadii(sz-2).linklength;
else
    critRadii=linklength(1);
end
critRadii=sort([abs(critRadii-newLinkLength(sz)), critRadii+newLinkLength(sz)]);
CollisionVarCritRadii(sz-1).linklength=critRadii;

%% apply the RandomLoopGenerator algorihm
out=RandomLoopGenerator(newLinkLength,CollisionVarCritRadii,nsamples,initAngle);
numConfig=size(out,2);

%% special case, return null
if numConfig==0
    leftConfig=[];
    endPt=[];
    return;
end

%% otherwise
leftConfig=out(1:sz-1,numConfig);

%% calculate the end point by linklength(1) ... linklength(sz-1); note:leftEnd is 2 *
%nsamples vectors, and each column is one of the possible end point of the
%chain of linklength(1) to linklength(sz-1)
leftEnd=fwd_kin_R(linklength(1:sz-1)',leftConfig,false);
newVec=pt - leftEnd;
leftConfig(sz,:)=atan2(newVec(2,:),newVec(1,:));

endPt= leftEnd+ [linklength(sz) * cos(leftConfig(sz,:)); linklength(sz) * sin(leftConfig(sz,:))];

