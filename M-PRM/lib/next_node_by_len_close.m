function [nearest_node] = next_node_by_len_close(temp_world_ang, mpData)
   nearest_node = [];
   if mpData.nVertices == 0
       return;
   end
   link_pos_temp =generate_link_position_close(temp_world_ang, mpData);
           
   dist_vec =  sum((squeeze(mpData.position(:, 1, 1:mpData.nVertices)) - repmat(link_pos_temp(:,1), 1, mpData.nVertices)) .^ 2) ...
                + sum((squeeze(mpData.position(:, 2, 1:mpData.nVertices)) - repmat(link_pos_temp(:,2), 1, mpData.nVertices)) .^ 2);
   
   [~, ind] = sort(dist_vec);
   if mpData.nVertices > mpData.conf.numCloses
       nearest_node = ind(1:mpData.conf.numCloses);
   else
       nearest_node = ind(1: mpData.nVertices);
   end
end