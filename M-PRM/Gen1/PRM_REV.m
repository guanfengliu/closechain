function [roadmap2, bound_samples, obst_samples, sampleTime,mapTime] = PRM_REV (roadmap1,RandomSample, Dist, LocalPlanner, k)

%%   RandomSample : A function that returns a random sample in freespace
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples
%
%   LocalPlanner :  A function that determines whether there is a collision
%        free straight line path between two points in configuration space
%
%   nsamples : The number of random samples to generate (removed in this
%   new function)
%
%   k : The number of neighbors that should be considered in
%        forming the roadmap graph.
%
% Output :
%   roadmap - a structure the samples, the edges and edge lengths in the
%        roadmap graph
%% Author: Leon G.F. Liu  09/23/2019
tic;
[samples,bound_samples, obst_samples] = RandomSample();
sampleTime=toc;
%fprintf(1,'Random sampling takes = %f sec \n', sampleTime);

samples = [samples,bound_samples,obst_samples];
nsamples=size(samples,2);
% Array of random samples, each column corresponds to the coordinates of a point in configuration space.
% repmat returns copies of an array
%%samples = repmat(x(:), 1, nsamples);
 
nOldSamples=roadmap1.nsamples;
oldSamples=roadmap1.samples;

edge_lengths=roadmap1.edge_lengths;
edges=roadmap1.edges;
nedges=size(edges, 1);

%% merge samples together
samples = [oldSamples, samples];
% edges - an array with 2 rows each column has two integer entries (i, j) which encodes the fact that sample i and sample j are connected by an edge
%%edges = zeros(nsamples*k, 2);
%%edge_lengths = ones(nsamples*k,1) * 999999999999;
 
% nedges - this integer keeps track of the number of edges we
% have in the graph so far
%%x=samples(:,1);
tic;
if nOldSamples > 0
    startIndex=1;
else
    startIndex=2;
end
for i = startIndex:nsamples
    % Note that we are assuming that RandomSample returns  nsample in
    % freespace
    curIndex=i+nOldSamples;
    y=samples(:,curIndex);
    
    % Find the nearest neighbors
    distances = Dist(y, samples(:,1:(curIndex-1)));

    length_n = length(distances);
    [sorted_distances, index_distances] = sort(distances);

    for ii = 1: min(k, length_n)
% %       if sorted_distances(ii) < 1
        j = index_distances(ii);
        [out,traj_out,dist]=LocalPlanner(y, samples(:,j));
        if (out)
            nedges = nedges + 1;
            edges(nedges,:) = [length_n+1, j];
            edge_lengths(nedges) = dist; %%sorted_distances(ii)
% %             nedges = nedges + 1;
% %             edges(nedges,:) = [j,length_n+1];
% %             edge_lengths(nedges) = dist; %%sorted_distances(ii);
        end
% %       else
% %           break;
% %       end
    end
    fprintf (1, 'nsamples = %d, nedges = %d\n', curIndex, nedges);
end
mapTime=toc;
roadmap2.samples = samples;
roadmap2.edges = edges(1:nedges, :);
roadmap2.edge_lengths = edge_lengths(1:nedges);
roadmap2.nsamples=nsamples + nOldSamples;
