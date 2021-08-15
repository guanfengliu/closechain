function [traj_out, dist, numCCs] = ProjectMap(x1, nearest, mpData)
%% project x1 onto the free velocity cone of the nearest neighbor
%% and then connect the projected sample to nearest.
%@x1, a random sample
%@nearest, the nearest neighbor
%@mpData, the data structure containing robot kinematics
%% Author: Leon G.F. Liu  09/23/2019
numCCs = 0;
dist = 0;
traj_out = [];
numLinks = mpData.num_links - 3;

%% refer to RRV algoirthm
[minDist, isNarrow, narrowVertices, narrowEdges, narrowLinks] = calNarrowNessClose(nearest, mpData);
if (isNarrow)
   % step 2, compute the Jacobian, and escape direction, set
   % up the start link index
   start_link_index = 1;
   numNarrowLinks = length(narrowLinks);
   % totalV is used to calculate projection space
   totalV = zeros(2 * numNarrowLinks, numNarrowLinks);
   % entire Jacobian that assembles jacobians for all narrow
   % links
   % totalJ * thetadot = totalV * alpha,  alpha is
   % numNarrowLinks * 1 column vector
   totalJ = [];
   % CJ is the current jacobian upto current narrow link
   cJ = [];
   % null space of cJ
   NJ =[];
   for i=1:numNarrowLinks,
       ll_vector = mpData.conf.linkLengthVec(start_link_index:narrowLinks(i));
       jt_vector = nearest(start_link_index:narrowLinks(i))';
       % jacobian matrix for the link segment form
       % start_link_index to narrowLinks(i) only
       tJ = [-ll_vector.*sin(jt_vector); ll_vector.*cos(jt_vector)];
       %NJ = blkdiag(NJ, null(tJ));
       cJ = [cJ, tJ];
       % zero2 is 2 * (num_links-narrowLinks(i)) matrix to
       % represent that the motion of the chain from
       % narrowLinks(i) to num_links does not affect velocity
       % of narrowLinks(i)
       zero1 = zeros(1, numLinks - narrowLinks(i));
       zero2 = [zero1;zero1];
       totalJ = [totalJ; cJ, zero2];
                   
       % now find out vector of feasible linear speed of narrow
       % links
       narrowEdge = narrowEdges(i, :);
       pt = mpData.obstacle.coord(:, narrowEdge);
       totalV(2 * (i-1) + 1: 2 * i, i) = pt(:,2) - pt(:,1); % medium axis direction
    end
    % now compute the projection matrix
    NJ = null(totalJ);
    AA = totalJ * totalJ';
    if det(AA) < 0.0001  % in singularity
       A = NJ;   % move in null space, trying to escape form singularity
    else
       A=[totalJ' * inv(AA) * totalV, NJ];
    end
    % now perform projection of sample
    if size(A,1)~= 9 || size(x1,1)~=12 || size(nearest,1)~=12
        fprintf(1, 'size A= %d, %d, size(x1)=%d, size(nearest)=%d', size(A,1), size(A,2), size(x1,1), size(nearest,1));
    end
    dx = x1 - nearest;
   
    tmp_sample = nearest(1:numLinks) + A* inv(A'*A) * A' * dx(1:numLinks);
                
    dist = norm(ConvertNormal(tmp_sample - nearest(1:numLinks)));
    totalSamples = min(dist / mpData.conf.step_delta, mpData.conf.rrv_narrow_numSamples);
    tmp = nearest(1:numLinks) + ConvertNormal(tmp_sample - nearest(1:numLinks)) * [1:1:totalSamples-1]/totalSamples; 
    for i=1:size(tmp,2)
        tmp1 = tmp(:,i);
        % compute the joint angles of the last two links
        tip = [sum(mpData.conf.linkLengthVec(1:numLinks).*cos(tmp1'));
               sum(mpData.conf.linkLengthVec(1:numLinks).*sin(tmp1'))];
        pr = [mpData.conf.linkLengthVec(end), 0]';
        [interv] = cal_ik_2pt(mpData.conf.linkLengthVec(end-2), mpData.conf.linkLengthVec(end-1), tip, pr);
        if size(interv, 2) > 0
            tmp1 = [tmp1; interv(:, 1); pi];
            numCCs = numCCs + 1;
            if  ~CheckCollision(tmp1, mpData, true)
               traj_out = [traj_out, tmp1];
            else
               dist = dist * (i-1) /mpData.conf.rrv_narrow_numSamples;
               return;
            end
        end
    end
end
end
