function [minDist, isNarrow, narrowVertices, narrowEdges, narrowLinks] = calNarrowNessClose(joint, mpData)
   narrowVertices=[]; narrowEdges=[]; narrowLinks = [];
   link_pos = generate_link_position_close(joint, mpData);
   minDist = inf;
   isNarrow = false;
   for i=1:mpData.num_links-3,
       [dist, isNarrowTemp, narrowVertex, narrowEdge] = determineNarrowDistClose(link_pos(i,:)', link_pos(i+1,:)', mpData);
        if ~isNarrow && isNarrowTemp
           narrowVertices = [narrowVertices, narrowVertex];
           narrowEdges = [narrowEdges; narrowEdge];
           narrowLinks = [narrowLinks, i];
           isNarrow = true;
         end
         if dist < minDist
            minDist = dist;
         end
    end
end