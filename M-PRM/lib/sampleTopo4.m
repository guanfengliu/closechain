function [topo_samples] = sampleTopo4(mpData)
topo_samples = [];
   %numCCs = 0;
for numTop=1:mpData.conf.toposamples,
   single_sample = [];
   if mpData.num_narrow_pairs > 0        
     [narrowLinkSet, narrowPairSet] = computeNarrowLinkSetClose(mpData);  
     num_narrow_links = length(narrowLinkSet);
     base_terminal = [0,0]'; %top_state.base;
     base_link = 1;
     if num_narrow_links > 0
        for i=1:num_narrow_links
           % which narrow vertex-edge pair
           narrowPair = narrowPairSet(i);
           narrowEdge = mpData.narrow_faces(narrowPair, :);
           narrowVertex = mpData.narrow_vertices(narrowPair);
           % pt(1:2,:) is the narrowEdge
           pt = mpData.obstacle.coord(:, narrowEdge);
                    % pt(3,:) is the narrow vertex
           pt = [pt, mpData.obstacle.coord(:, narrowVertex)];
                 
           pt_l = 0.5 * (pt(:, 3) + pt(:, 1));
           pt_r = 0.5 * (pt(:, 3) + pt(:, 2));
           centerPt = pt_l + rand(1) * (pt_r - pt_l);  %pt * (randNumber/sum(randNumber));
                           
           dpt = pt(:, 2) - pt(:, 1);
           % this is median axis direction for avoiding collision
           angle = atan2(dpt(2), dpt(1));
                    
            %% narrow link index  n_link
           n_link = narrowLinkSet(i);
                    % randomly chosen the fraction of the link n_link that
                    % points from centerPt to pt_r
           frac = rand(1);
           linkVec = mpData.conf.linkLengthVec(n_link) * [cos(angle); sin(angle)];
           % randomly choose directions 
           dirRand = rand(1);
           if dirRand > 0.5  %  frac *  linkVec  points toward pt_r,  (1-frac) * linkVec pointers from center towrad pt_l
             narrow_endPts = centerPt + [frac * linkVec,-(1-frac) * linkVec] ;
           else
             narrow_endPts = centerPt - [frac * linkVec,-(1-frac) * linkVec] ;
           end
                    %% now create open or closed chains to apply IK for sampling their configurations.
           dvec = narrow_endPts(:, 1) - base_terminal;
           tmpData.linklength = [mpData.conf.linkLengthVec(base_link:n_link-1), norm(dvec)];
           base_link = n_link + 1;
           base_terminal = narrow_endPts(:, 2);

           [CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(tmpData.linklength);
           tmpData.init_angle = atan2(dvec(2), dvec(1));
           tmpData.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
           tmpData.CollisionVarCritRadiusR = CollisionVarCritRadiusR;
           % @enable_bound, do we also smaple the boundary variety
           tmpData.enable_bound = false;
           tmpData.left_2_right = true;
           tmpData.nonbsamples = 1;
           [out] = RandomLoopGeneratorGen2(tmpData);
            % if any of the subchain has no IK solution, return
           if size(out, 2) == 0
              single_sample = [];
              break;
           end
           dvec1 = narrow_endPts(:,2) - narrow_endPts(:,1);
           angle=atan2(dvec1(2), dvec1(1));
           single_sample = [single_sample; [out(1:end-1,1); angle]];
        end
                 %remaining piece of chain, which is purely open, just randomly
                 %sample
        if size(single_sample,1)==0  % subchain IK fails for narrow links, then continue next sample
            continue;
        end
                       %% now create open or closed chains to apply IK for sampling their configurations.
        dvec = [mpData.conf.linkLengthVec(end), 0]' - base_terminal;
        tmpData.linklength = [mpData.conf.linkLengthVec(base_link:mpData.num_links-1), norm(dvec)];
        [CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(tmpData.linklength);
        tmpData.init_angle = atan2(dvec(2), dvec(1));
        tmpData.CollisionVarCritRadiusL = CollisionVarCritRadiusL;
        tmpData.CollisionVarCritRadiusR = CollisionVarCritRadiusR;
                       % @enable_bound, do we also smaple the boundary variety
        tmpData.enable_bound = false;
        tmpData.left_2_right = true;
        tmpData.nonbsamples = 1;
        [remain_chain] = RandomLoopGeneratorGen2(tmpData);
                       % if any of the subchain has no IK solution, return
                       % and clear topo_state
         if size(remain_chain,2)==0
              single_sample = [];
              continue;   % continue next sample
         end 
         remain = remain_chain(:,1);
         single_sample = [single_sample; remain(1:end-1); pi];
         drawConfig4(mpData.conf.linkLengthVec, single_sample, mpData.obstacle, mpData.conf.thickNess, 'red');
         topo_samples = [topo_samples, single_sample];
     end
   end
end
end