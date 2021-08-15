function [dist, isNarrow, narrowVertices, narrowEdges] = determineNarrowDistClose(pp1, pp2, mpData)
            narrowVertices = [];
            narrowEdges = [];
            numNarrowVertex = length(mpData.narrow_vertices);
            % check every narrow vertex-face pairs for possible closeness
            dist = inf;
            isNarrow = false;
            for i=1:numNarrowVertex,
                narrow_vertex = mpData.narrow_vertices(i);
                qq = mpData.obstacle.coord(:, narrow_vertex);
                narrow_edge = mpData.narrow_faces(i,:);
                qq1 = mpData.obstacle.coord(:, narrow_edge(1));
                qq2 = mpData.obstacle.coord(:, narrow_edge(2));
                
                dist_qq = computeDistVertex2EdgeClose(qq, pp1, pp2, 0.5 * mpData.conf.thickNess);
                dist_21 = computeDistVertex2EdgeClose(qq1, pp1, pp2, 0.5 * mpData.conf.thickNess);
                dist_22 = computeDistVertex2EdgeClose(qq2, pp1, pp2, 0.5 * mpData.conf.thickNess);
                dist_23 = computeDistVertex2EdgeClose(pp1, qq1, qq2, 0.5 * mpData.conf.thickNess);
                dist_24 = computeDistVertex2EdgeClose(pp2, qq1, qq2, 0.5 * mpData.conf.thickNess);
                dist_qq12 = min([dist_21,dist_22,dist_23,dist_24]);
                
                % also need to check if qq and qq_1 (or qq_2) lies in the
                % two side of pp1-pp2 line segment
                norm_p12 = [pp1(2)-pp2(2), pp2(1) - pp1(1)]; %row vector
                vv1 = norm_p12 * (qq-pp1);
                vv2 = norm_p12 * (qq1-pp1);
                vv3 = norm_p12 * (qq2-pp1);
                in_opsite = false;
                if vv1 * vv2 < 0 && vv1 * vv3 < 0
                    in_opsite = true;
                end
                
                dist_t = max(dist_qq, dist_qq12); %min(dist_qq, dist_qq12);
                if dist_t < dist,
                    dist =  dist_t;
                end
                if dist_qq < mpData.conf.narrow_link_dist  && dist_qq12 < mpData.conf.narrow_link_dist && in_opsite && ~isNarrow
                    narrowVertices = [narrowVertices, narrow_vertex];
                    narrowEdges =[narrowEdges; narrow_edge];
                    isNarrow = true;
                end
            end
        end