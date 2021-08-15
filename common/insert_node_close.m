function [mpData, succ] = insert_node_close(neighbors, lens, new_node, mpData)
    % method insert new node in the tree
     succ = true;
     new_node_ind = mpData.nVertices + 1;
     if new_node_ind > mpData.max_nodes
         succ = false;
         return;
     end
     mpData.total_samples(:, new_node_ind) = new_node;
     mpData.nVertices = mpData.nVertices + 1;
     [mpData] = update_link_position_close(new_node_ind, mpData);
     for i=1:length(neighbors)
        mpData.nedges = mpData. nedges + 1;
        mpData.edges(mpData.nedges,:) = [new_node_ind, neighbors(i)];
        mpData.edge_lengths(mpData.nedges) = lens(i);
     end
end