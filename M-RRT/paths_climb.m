function patharray = paths_climb_good(adj, src, snk, coord, verbose)
%PATHBETWEENNODES Return all paths between two nodes of a graph
%
% pth = pathbetweennodes(adj, src, snk)
% pth = pathbetweennodes(adj, src, snk, vflag)
%
%
% This function returns all simple paths (i.e. no cycles) between two nodes
% in a graph.  Not sure this is the most efficient algorithm, but it seems
% to work quickly for small graphs, and isn't too terrible for graphs with
% ~50 nodes.
%
% Input variables:
%
%   adj:    adjacency matrix
%
%   src:    index of starting node
%
%   snk:    index of target node
%
%   vflag:  logical scalar for verbose mode.  If true, prints paths to
%           screen as it traverses them (can be useful for larger,
%           time-consuming graphs). [false]
%
% Output variables:
%
%   pth:    cell array, with each cell holding the indices of a unique path
%           of nodes from src to snk.

% Copyright 2014 Kelly Kearney, and revised by Leon Guanfeng Liu by adding
% priority (weights) into the algorithm

if nargin < 4
    verbose = false;
end

n = size(adj,1);
%% for applying astar-algorithm
% gcost = realmax * ones(1,n)
%% fcost = zeros(1,n);
%% parent of current node
%% parent = zeros(1,n);
pred = src;
%% stack is for tracking through all paths
stack = src;   %.push(src, 0);
% src;

% stop = false;

cycles = cell(0);

next = cell(n,1);
for in = 1:n
    next{in} = find(adj(in,:));
end

visited = cell(0);

% pred = src;
% gcost(src) = 0;
% for storing all paths
patharray = cell(0);
while 1
    
    visited = [visited; sprintf('%d,', stack)];
    
    [stack, pred] = addnode(adj, stack, next, visited, pred, snk, coord);
    if verbose
        fprintf('%2d ', stack);
        fprintf('\n');
    end
    
    if isempty(stack)
        break;
    end
    
    if stack(end) == snk
        patharray = [patharray; {stack}];
        visited = [visited; sprintf('%d,', stack)];
        stack = popnode(stack);
    elseif length(unique(stack)) < length(stack)
        cycles = [cycles; {stack}];
        visited = [visited; sprintf('%d,', stack)];
        stack = popnode(stack);
    end

end

end
function [stack, pred] = addnode(adj, stack, next, visited, pred, snk, coord)
  top = stack(end);
  newnode = setdiff(next{top}, pred);
  possible = arrayfun(@(x) sprintf('%d,', [stack x]), newnode, 'uni', 0);
  
  isnew = ~ismember(possible, visited);
  if any(isnew)
     num_isnew = length(isnew);
     leastcost =realmax;
     for i=1:num_isnew
        if isnew(i)       
          childNode = newnode(i);
          fcost = adj(top, childNode) + norm(coord(:,childNode), coord(snk));
          if fcost < leastcost
              leastcost = fcost;
              leastNode = childNode;
          end
        end
     end
     stack = [stack, leastNode];
        %path = str2num(isnew(possible{neighborIndex(top1)}));
     pred = top;
  else
    [stack, pred] = popnode(stack);
  end
end

function [stack, pred] = popnode(stack)

  stack = stack(1:end-1);
  if length(stack) > 1
    pred = stack(end-1);
  else
    pred = [];
  end
end



    
    
    



   
    
        