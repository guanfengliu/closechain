 function [narrowLinkSet, narrowPassageSet] = computeNarrowLinkSetClose(mpData) %, mode)  %, terminal)
  
    narrowLinkSet = [];
    narrowPassageSet = [];
    minNarrowLink = 3;
    maxNarrowLink = mpData.num_links - 3;
    
          
    % first narrow link
   startNarrowLink = randi([minNarrowLink, maxNarrowLink], 1);
   narrowLinkSet = [startNarrowLink];
   while narrowLinkSet(end) + 3 <= maxNarrowLink 
        nextlink = randi([narrowLinkSet(end)+3, maxNarrowLink],1);
        narrowLinkSet = [narrowLinkSet, nextlink];
   end
   num_narrow_links = length(narrowLinkSet);
   start_narrow_pair = 1;
   %% for each narrow link chose correspongding narrow vertex_edge pairs
   for i=1:num_narrow_links
     pair_index = randi([start_narrow_pair, mpData.num_narrow_pairs], 1);
     narrowPassageSet = [narrowPassageSet, pair_index]; %randi(mpData.num_narrow_pairs, num_narrow_links, 1);
     if pair_index <  mpData.num_narrow_pairs
         start_narrow_pair = pair_index + 1;
     end
   end
   
   if length(narrowPassageSet) < num_narrow_links
      narrowLinkSet = [];
      narrowPassageSet = [];
   end
end