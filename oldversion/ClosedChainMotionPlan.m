function ClosedChainMotionPlan(linklength,obst,start_config,end_config,thickNess,nsamples,num_internal_loops,neighbors)
%% @brief this is the main function to plan motion between start and end config given
%% a closed chain based at (0,0), while ended at (lm,0); 
%% @param linklength is a row vector of linklengths (l1,l2,...,lm)
%% @param obst is n by 2 matrix, each column is the coordinate of a point obstacle
%% @start_config is a row vector of size 1 * m
%% @end_config is a rwo vector of size 1 * m
%% @thickNess is the scalar which is mainly for drawing the links of chain, and also for collision checking
%% @nsamples, number of samples generated in each iteration of random sampling algorithm
%% @num_internal_loops, number of iterations for calling random sampling algorithm
%% @neighbors, for each collision free milestone, how many close neighbors to connect using local planner 
%% Author: Leon G.F. Liu  09/23/2019

num_link=length(linklength);  %% number of links
if num_link < 4
    fprintf(1,'C-space is not a continuous space if number of links £¨including base) < 4 \n');
    return;
end
num_obsz=size(obst,2);  %% number of obstacle

%% draw the start and end configuration
fv = closedchainthick (linklength,start_config,thickNess);
fv2 = closedchainthick (linklength,end_config,thickNess);
figure(1)
p = patch (fv);
hold on
p.FaceColor = 'green';
p.EdgeColor = 'none';

p2 = patch(fv2);
p2.FaceColor = 'red';
p2.EdgeColor = 'none';



%% need to generate the structure of boundary variety
%% Recall B(1)= f1^{-1}(l1+l2 circle + abs(l1-l2) circle)
%% recursively, we can deduce B(2) = f2^{-1}£¨l1+l2+l3 circle, l1+l2-l3 circle, l1-l2+l3 circle,
%% l1-l2-l3 circle)
%% data structure for storing B(1) ... B(n-3) critical circle lengths

oldLengthVec=[linklength(1)];
for i=2:num_link-1
    newLengthVec=[abs(oldLengthVec-linklength(i)),oldLengthVec + linklength(i)];
    CollisionVarCritRadiusL(i-1).linklength=sort(newLengthVec);
    oldLengthVec=CollisionVarCritRadiusL(i-1).linklength;
end

oldLengthVec=[linklength(num_link-1)];
for i=num_link-2:-1:1
    newLengthVec=[abs(oldLengthVec-linklength(i)),oldLengthVec + linklength(i)];
    CollisionVarCritRadiusR(num_link-1-i).linklength=sort(newLengthVec);  
    oldLengthVec=CollisionVarCritRadiusR(num_link-1-i).linklength;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% handling the set of point obstacles: (1) dilate each point obstacle
% (2) sampling points on the bounary of the dilated circle
% (3) use convex hull of these sampling points for collision checking
dilate_epsilon=0.05; % dilate_epsilon is used to replace each point obtacle by a polygonal obstacle for collision checking
                     % 2 * dilate_epsilon is used to sampling region near
                     % C-obstacle
numApp=4; %% number of approximating point obtacle for each original point obstacle
angles=-pi:2*pi/numApp:pi;
angles=angles(1:numApp);
vert=dilate_epsilon * [cos(angles);sin(angles)];
% the set of pt obstacles after dilation, each column becomes an
% approximating point obstacle, used for sampling regions near C-obst
dilatePtObst=[];  
polyObst.vertices=[];  % vertices and faces triangulation of all obstacle conex hulls, used for collision checking
polyObst.faces=[];
for i=1:num_obsz,
    plot(obst(1,i),obst(2,i),'b*')
    dia=obst(:,i) + 2 * vert;
    dilatePtObst(:,(i-1)*numApp+1:i*numApp)=dia; 
    plot(dia(1,:),dia(2,:),'b.')
    polyObst = appendFV(polyObst,TriangPolygon(obst(:,i)+vert)); 
end
patch(polyObst); % drawing all these approximating obstacle (convex hulls)


%% Build roadmap

%in the following PRM function: there are 8 inputs 
%@linklength, a vector of link lengths of n-link close-chain
%@thickNess, thickness of links for collision checking
%@CollisionVarCritRadiusL: (L means that the
%critical circle radius is calculated from the left end of the closed-chain
%@CollisionVarCritRadiusR: the set of critical cricle radius calculated
%from the right end of the close-chain
%@polyObst: the dilated obstacle (approximated by set of points on the
%boundary, and in turn approximated by the polygon of the points, so the name polyObst)
%@dilatePtObst: the set of approximing points on the boundary of the approximating disks of each point obstacle 
%@nsamples: number of samples to be generated in each run of algorithm in
%RandomSampleCloseChain function
%@num_internal_loops: how many rns for each algorithm in
%RandomSampleCloseChain
roadmap = PRM (@()(RandomSampleClosedChain(linklength,thickNess,CollisionVarCritRadiusL,CollisionVarCritRadiusR,polyObst,dilatePtObst,nsamples,num_internal_loops)), @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,linklength,thickNess)), neighbors);

% Add start, end nodes
roadmap2 = AddNode2PRM (start_config', roadmap, @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,linklength,thickNess)), 10);
roadmap2 = AddNode2PRM (end_config', roadmap2, @DistClosedChain, @(x,y)(LocalPlannerClosedChainSimple(x,y,polyObst,linklength,thickNess)), 10);

%% Plan a route using Dijkstra algorithm
fprintf (1, 'nsamples = %d \n', roadmap.nsamples);
SS=roadmap2.edges(:,1)';
TT=roadmap2.edges(:,2)';
weights=roadmap2.edge_lengths';
edges=roadmap2.edges;
samples=roadmap2.samples;
edge_lengths=roadmap2.edge_lengths;
save('roadmap2_edges.txt','edges','-ascii');
save('roadmap2_samples.txt','samples','-ascii');
save('roadmap2_length.txt','edge_lengths','-ascii');
G = graph(SS,TT,weights);
route = shortestpath(G,roadmap.nsamples+1, roadmap.nsamples+2);


% Generate the resulting paths from the path on graph using local planner
% again
final_traj=[];
for i = 2:length(route)
   x1 = roadmap2.samples(:,route(i-1));
   x2 = roadmap2.samples(:,route(i));
   [out,traj_out] = LocalPlannerClosedChainSimple (x1, x2, polyObst,linklength,thickNess); 
   if (out)
       final_traj=[final_traj,traj_out];
   else
      fprintf(1,'Ouch\n'); 
      return;   
   end 
end
num_sample=size(final_traj,2);
if (num_sample>0)
  %%save traj data file and print paths in workspace
  save('test_data_10obst_example.txt','final_traj','-ascii');
  make_print(final_traj,linklength,obst,dilatePtObst);
else
    fprintf(1,'path doesnt exist! need more samples \n');
end