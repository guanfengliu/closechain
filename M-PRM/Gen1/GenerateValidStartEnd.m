function [start_config,end_config]=GenerateValidStartEnd(linklength,thickNess,CollisionVarCritRadiusL,polyObst,obst, dilatePtObst,fig_hnd,noTrial)
%% @brief this is the main function to plan motion between start and end config given
%% a closed chain based at (0,0), while ended at (lm,0); 
%% @param linklength is a row vector of linklengths (l1,l2,...,lm)
%% @param obst is n by 2 matrix, each column is the coordinate of a point obstacle
%% @thickNess is the scalar which is mainly for drawing the links of chain, and also for collision checking 
%% Author: Leon G.F. Liu  09/23/2019

%% Random sample two configuration
start_config=RandomSampleClosedChainSingle(linklength,thickNess,CollisionVarCritRadiusL,polyObst,1);
%start_config=start_config';
end_config=RandomSampleClosedChainSingle(linklength,thickNess,CollisionVarCritRadiusL,polyObst,0);
%end_config=end_config';
%% draw obst
plot(obst(1,:),obst(2,:),'b*');
hold on;
plot(dilatePtObst(1,:),dilatePtObst(2,:),'b.');
hold on;
%% draw the start and end configuration
thick=0.05;
fv = closedchainthick (linklength,start_config,thick);
fv2 = closedchainthick (linklength,end_config,thick);
figure(fig_hnd);
hold on
p = patch (fv);
hold on
p.FaceColor = 'green';
p.EdgeColor = 'none';

p2 = patch(fv2);
p2.FaceColor = 'red';
p2.EdgeColor = 'none';
axis equal;
xlabel('x');
ylabel('y');
str=['No.', num2str(noTrial), 'pair of start and goal configurations'];
title(str);