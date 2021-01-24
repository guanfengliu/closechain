function outPrm = AddNode2PRM(x, kdtree, prm, LocalPlanner, neighbors)
%% Add a node to a PRM - used to attach start and destination configurations
%% Author: Leon G.F. Liu  09/23/2019
x= ConvertNormal(x);
n = prm.nsamples;
samples = prm.samples;
IdxNN = knnsearch(kdtree,x','K',neighbors); %,'Distance',@DistClosedChainKnn);  %% the reason for k+1 is to remove the point itself as the closest neighbor 

edges = zeros(neighbors,2);
edge_lengths = zeros(1,neighbors);

nedges = 0;

sz_c = length(IdxNN);
for i = 1:sz_c,
    ind_y=IdxNN(i);
    y=samples(:,ind_y);  %% target point
   
   [out,traj_out,dist]=LocalPlanner(y, x);
   if (out)
       nedges = nedges + 1;
       edges(nedges,:) = [n+1, ind_y];
       edge_lengths(nedges) = dist;
%        nedges = nedges + 1;
%        edges(nedges,:) = [ind_y, n+1];
%        edge_lengths(nedges) = dist;
    end
    fprintf (1, 'nsamples = %d, nedges = %d\n', i, nedges);
end
fprintf (1, 'neighbors = %d, nedges = %d\n', neighbors, nedges);
if nedges > 0
  outPrm=[]; 
  outPrm.samples = [prm.samples, x];
  outPrm.edges = [prm.edges; edges(1:nedges, :)];
  outPrm.edge_lengths = [prm.edge_lengths,edge_lengths(1:nedges)];
  outPrm.nsamples = n + 1;
else
    outPrm = prm;
end