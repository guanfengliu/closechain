function [outMpData] = PRM(outMpData, RandomSample, LocalPlanner, k)
%% generation 2 Probabilistic Roadmap Algorithm
%% @outMpData, a data structure that include roadmap sub-structure, obst-region samples, non-obst samples, boundary-variety samples,
% sampling time, and roadmap computation time
%%   RandomSample : A function that returns random samples in freespace, which
% should be replaced by the actual function that implements various
% algorithms for capturing narrow passages in high-dimensional space
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples£»
%   this function may be deprecated by using standard euclidean, or
%   Minkowski metric (in the future maybe just metric type
%
%   LocalPlanner :  A function that determines whether there is a collision-free 
% straight line path between two points in configuration space
%
%   k : The number of neighbors that should be considered in
%        forming the roadmap graph.
%
% Output :
%   outMpData.roadmap - a structure the represent the map, including edges
%   and edge lengths, and vertices
%% Author: Leon G.F. Liu  09/23/2019

tic;
[samples, bound_samples, obst_samples] = RandomSample();
outMpData.sampleTime=toc;
outMpData.regular_samples = [outMpData.regular_samples,samples];  %% 
outMpData.bound_samples = [outMpData.bound_samples, bound_samples];
outMpData.obst_samples = [outMpData.obst_samples, obst_samples];

%% new samples created in this iteration
samples = [samples,bound_samples,obst_samples];

%% get the roadmap data obtained in the previous iteration
roadmap1 = outMpData.roadmap; 
oldSamples=roadmap1.samples;
edge_lengths=roadmap1.edge_lengths;
edges=roadmap1.edges;
nedges=size(edges, 1);

%% merge samples together
samples = [oldSamples, samples];

tic;
%% create kd-tree for later on applying nearest neighbor search
X=samples'; %% each row of X is a random sample
%%kdtree =createns(X,'NSMethod','exhaustive','Distance',@DistClosedChainKnn);  %%
kdtree= KDTreeSearcher(X,'Distance','minkowski','BucketSize', 2*k);  %% we make sure the bucket size is a little bit high than k 
outMpData.kdtree = kdtree;
%% find the closest k+1 neighbor for each point in X 
%%IdxNN = knnsearch(kdtree,X,'K',k+1,'Distance',@DistClosedChainKnn);  %% the reason for k+1 is to remove the point itself as the closest neighbor 
IdxNN = knnsearch(kdtree,X,'K',k+1);
sz_IdxNN = size(IdxNN);
sz_r = sz_IdxNN(1);
sz_c = sz_IdxNN(2);
for i = 1:sz_r,
    y=samples(:,i);  %% target point
    
    % check all k nearest neighbors
    for j=2:sz_c,  %% recall j=1 is the y itself   
      ind_y = IdxNN(i,j); 
      [out,traj_out,dist]=LocalPlanner(y, samples(:,ind_y));
      if (out)
            nedges = nedges + 1;
            edges(nedges,:) = [i, ind_y];
             edge_lengths(nedges) = dist;
%             nedges = nedges + 1;
%             edges(nedges,:) = [ind_y, i];
%             edge_lengths(nedges) = dist;
      end
    end
    fprintf (1, 'nsamples = %d, nedges = %d\n', i, nedges);
end
mapTime=toc;
roadmap2 = [];
roadmap2.samples = samples;
roadmap2.edges = edges(1:nedges, :);
roadmap2.edge_lengths = edge_lengths(1:nedges);
roadmap2.nsamples= size(samples,2);
outMpData.roadmap = roadmap2;
outMpData.mapTime = mapTime;

