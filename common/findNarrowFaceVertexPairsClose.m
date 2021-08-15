        %% find narrow face-vertex pairs 
        function [mpData]= findNarrowFaceVertexPairsClose(mpData)
            % vertice matrix of the vertices from all obstacles, rows are coordinates of vertices
            %mpData.obstacle.coord = [];
            tempArray = [];
            total_vertices = 0;
            % next 
             for i=1:mpData.obstacle.numObst 
                %mpData.obstacle.coord = [mpData.obstacle.coord, mpData.obstacle.obstDesc{i}];
                tempArray = [tempArray, size(mpData.obstacle.obstDesc{i}, 2)];
            end
            total_vertices = sum(tempArray);
            vertArray = cumsum(tempArray);
            % next is a cell array for computing the adjacency vertices
            next = cell(total_vertices, 1);
            % compute the normal and tangent of the edges containing a given vertex
            normal_next = cell(total_vertices, 1);
            tangent_next = cell(total_vertices, 1);
            vert_offset = 0;
            for i=1:mpData.obstacle.numObst,
               node_l = vert_offset + tempArray(i);
               node_c = vert_offset + 1;
               node_r = vert_offset + 2;
               next{node_c} = [node_l, node_r];
               [tl, tr, nl, nr] = computeTangentAndNormalClose(node_l, node_c, node_r, i, mpData);
               %% tangents and normals of the two edges adjcent to node_c
               tangent_next{node_c} = [tl, tr];
               normal_next{node_c} = [nl, nr];
               for j=2:tempArray(i)-1,
                   %next{vert_offset+j} = [vert_offset+j-1, vert_offset+j+1];
                   node_l = vert_offset+j-1;
                   node_c = vert_offset+j;
                   node_r = vert_offset+j+1;
                   next{node_c} = [node_l, node_r];
                   [tl, tr, nl, nr] = computeTangentAndNormalClose(node_l, node_c, node_r, i, mpData);
                   %% tangents and normals of the two edges adjcent to node_c
                   tangent_next{node_c} = [tl, tr];
                   normal_next{node_c} = [nl, nr];
               end
               node_l = vertArray(i)-1;
               node_c = vertArray(i);
               node_r = vert_offset+1;
               next{vertArray(i)} = [node_l, node_r];
               [tl, tr, nl, nr] = computeTangentAndNormalClose(node_l, node_c, node_r, i, mpData);
               %% tangents and normals of the two edges adjcent to node_c
               tangent_next{node_c} = [tl, tr];
               normal_next{node_c} = [nl, nr];
               vert_offset = vertArray(i);
            end
            
            %% using knn search to find the cloest neighbors
            %%kdtree =createns(X,'NSMethod','exhaustive','Distance',@DistClosedChainKnn);  %%
            kdtree= KDTreeSearcher(mpData.obstacle.coord','Distance','minkowski','BucketSize', 2 * mpData.conf.neighbors);  %% we make sure the bucket size is a little bit high than k 

            %% find the closest k+1 neighbor for each point in X 
            IdxNN = knnsearch(kdtree, mpData.obstacle.coord', 'K', mpData.conf.neighbors + 1);
            sz_IdxNN = size(IdxNN);
            sz_r = sz_IdxNN(1);
            sz_c = sz_IdxNN(2);
            %% interval_index determines which obtacle a vertex belong to
            interval_index = 1;
            left_index = 0;
            right_index = vertArray(interval_index);
            for i = 1:sz_r,
               if i > right_index
                   interval_index = interval_index + 1;
                   left_index = right_index;
                   right_index = vertArray(interval_index);
               end
               yy = mpData.obstacle.coord(:, i);  %% target point
               % check all k nearest neighbors, see if they can be reached
               % from y
               for j=2:sz_c,  %% recall j=1 is the y itself   
                  %%  if IdxNN(i,j) is not in the same polygon 
                  indx = IdxNN(i,j);
                  if indx <= left_index || indx > right_index
                     % find out the edges that include the vertex indx
                     next_l = next{indx}(1);
                     pc = mpData.obstacle.coord(:, indx);
                     pl = mpData.obstacle.coord(:, next_l);
                     dist = computeDistVertex2EdgeClose(yy, pl, pc, 0.5 * mpData.conf.thickNess);
                     if dist <= mpData.conf.narrow_dist  %% n
                         % narrow faces, but need to check if the vertices
                         % lies in the positive half plane of the narrow
                         % edge
                         positive = checkNormalDirectionClose(normal_next{indx}(:,1), yy-pc, mpData.conf.half_plane_epsilon);
                         if positive
                            mpData.narrow_faces = [mpData.narrow_faces; indx, next_l];
                            % narrow vertices
                            mpData.narrow_vertices=[mpData.narrow_vertices;i];
                         end
                     end
%                      next_r = next{indx}(2);
%                      pr = mpData.obstacle.coord(:, next_r);
%                      dist = computeDistVertex2EdgeClose(yy, pr, pc, 0.5 * mpData.conf.thickNess);
%                      if dist <= mpData.conf.narrow_dist  %% n
%                          positive = checkNormalDirectionClose(normal_next{indx}(:,2), yy-pc, mpData.conf.half_plane_epsilon);
%                          if positive
%                             mpData.narrow_faces = [mpData.narrow_faces; indx, next_r];
%                             % narrow vertices
%                             mpData.narrow_vertices=[mpData.narrow_vertices;i];
%                          end
%                      end
                  end
               end
            end
           mpData.num_narrow_pairs = length(mpData.narrow_vertices);
           fprintf (1, 'num narrow vertex-edge pair %d\n', mpData.num_narrow_pairs); 
        end