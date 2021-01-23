function roadmap = PRM (RandomSample, Dist, LocalPlanner, k)

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

samples = RandomSample();

nsamples=size(samples,2);
% Array of random samples, each column corresponds to the coordinates of a point in configuration space.
% repmat returns copies of an array
%%samples = repmat(x(:), 1, nsamples);
 
% edges - an array with 2 rows each column has two integer entries (i, j) which encodes the fact that sample i and sample j are connected by an edge
edges = zeros(nsamples*k, 2);
edge_lengths = zeros(nsamples*k, 1);
 
% nedges - this integer keeps track of the number of edges we
% have in the graph so far
nedges = 0;
 
x=samples(:,1);
for i = 2:nsamples
    % Note that we are assuming that RandomSample returns  nsample in
    % freespace
    y=samples(:,i);
    
    % Find the nearest neighbors
    distances = Dist(y, samples(:,1:(i-1)));

    length_n = length(distances);
    [sorted_distances, index_distances] = sort(distances);

    for ii = 1: min(k, length_n)
        j = index_distances(ii);
        [out,traj_out]=LocalPlanner(y, samples(:,j));
        if (out)
            nedges = nedges + 1;
            edges(nedges,:) = [length_n+1, j];
            edge_lengths(nedges) = sorted_distances(ii);
        end
    end
    fprintf (1, 'nsamples = %d, nedges = %d\n', i, nedges);
end
 
roadmap.samples = samples;
roadmap.edges = edges(1:nedges, :);
roadmap.edge_lengths = edge_lengths(1:nedges);
roadmap.nsamples=nsamples;
