function out = AddNode2PRM_REV (x, prm, Dist, LocalPlanner, neighbors)
%% Add a node to a PRM - used to attach start and destination configurations
%% Author: Leon G.F. Liu  09/23/2019

distances = Dist(x, prm.samples);

[sorted_distances, idx] = sort(distances);

n = length(distances);

edges = zeros(neighbors,2);
edge_lengths = zeros(1,neighbors);

nedges = 0;

for i = 1:min(50,n),   %%min(neighbors, n)
% %   if sorted_distances(i) < 1
    j = idx(i);
    [out1,traj_out,dist]=LocalPlanner(x, prm.samples(:,j));
    if (out1)
        nedges = nedges + 1;
        edges(nedges,:) = [n+1, j];
        edge_lengths(nedges) = dist; %%sorted_distances(i);
% %         nedges = nedges + 1;
% %          edges(nedges,:) = [j,n+1];
% %          edge_lengths(nedges) = dist; %%sorted_distances(i);
    end
% %   else
% %       break;
% %   end
end

fprintf (1, 'neighbors = %d, nedges = %d\n', neighbors, nedges);
if nedges > 0
  out.samples = [prm.samples, x];
  out.edges = [prm.edges; edges(1:nedges, :)];
  out.edge_lengths = [prm.edge_lengths,edge_lengths(1:nedges)];
  out.nsamples = prm.nsamples + 1;
else
    out = prm;
end