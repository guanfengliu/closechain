function [poly_dilate, polyObst] = processObstacle(obst,dilate_epsilon)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% obst.numObst -- number of obstcle
% obst.obstacles(i).coord =[X;Y] are the enveloping point set of the ith
% osbtacle; where X and Y are 1 by m row vector, if m =1, means point obstacle, if m > 1, convex or nonconvex nonpoint obstacles
% obst.obstacles(i).normal_vec =[X;Y] with each column denoting the normal vector at each vertex  obst.obstacles(i).coord(:,i)
% dilate_epsilon is the ratio of each samples growing along all samples on
% the boundary of obstacles.

%% note: we dilate each obstacle (including point, convex and non-convex obstacles) as following
% : (1) sampling curved or non-convex obstacles; for point or convex
% polygon, choose the vertices
% (2) computed the normal vectors attached to all samples
% (3) grow each sample along the normal by epsilon


%% this function dilate obstacles are represent them as planar convex or concave polygn
if nargin > 2 | nargin < 1
    error('processObstacle function expects at most 2 inputs, and at least 1 input');
else
    if nargin == 1
        dilate_epsilon = 0.01;  % default value
    end
end

%% initializing outputs
poly_dilate=[]; %% dilated obstacles, poly_dilate(i).coord 
numPoly = 0;
% obstacle triangulatin
polyObst.vertices=[];  % vertices and faces triangulation of all obstacle conex hulls, used for collision checking
polyObst.faces=[];

numObst = obst.numObst;
poly_dilate.numObst = numObst;
for i=1:numObst,
    %% the vertices of coord have to be arranged counter clockwise
    %% around obstacles
    coord = obst.obstacle(i).coord;
    normal_vec = obst.obstacle(i).normal_vec;  %% 
      
    dia=coord + 8 * dilate_epsilon * normal_vec;
    %% here poly_dilate is for dig out narrow passages
    poly_dilate.obst(i).coord = dia; 
    %% for each vertex,calculate the collision free interval of 
     % the orientation angle of a link, when it hits the vertex, but never gets
     % into the interior of the convex hull
    poly_dilate.obst(i).angle_interval = [];
    poly_dilate.obst(i).ext_vert = [];
    sz_vertex = size(dia,2);
    poly_dilate.obst(i).peri = 0;  %% compute dilate obstacle peri-meter, which is important to sample 
    for j=1:sz_vertex,
        ind_1 = j - 1;
        ind_2 = j;
        ind_3 = j + 1;
        ind_4 = j + 2;
        if ind_1 <= 0
            ind_1 = ind_1 + sz_vertex;
        end
        if ind_3 > sz_vertex
            ind_3 = ind_3 - sz_vertex;
        end
        if ind_4  > sz_vertex
            ind_4 = ind_4 - sz_vertex;
        end
        vec = dia(:,ind_1) - dia(:,ind_2);
        angle_left = atan2(vec(2),vec(1));
        poly_dilate.obst(i).peri = poly_dilate.obst(i).peri + norm(vec);
        
        vec = -dia(:,ind_3) + dia(:,ind_2);
        angle_right = atan2(vec(2),vec(1));
        if angle_right < angle_left
            angle_right = angle_right + 2*pi;
        end
        angle_intv = [angle_left;angle_right];
        poly_dilate.obst(i).angle_interval = [poly_dilate.obst(i).angle_interval, angle_intv];
        
        %% calculate external vert
        ext_vert= cal_intersec(dia(:,ind_1),dia(:,ind_2),dia(:,ind_3),dia(:,ind_4));
        poly_dilate.obst(i).ext_vert = [poly_dilate.obst(i).ext_vert, ext_vert];
    end
     
    %% polyObst is for collision checking
    polyObst = appendFV(polyObst,TriangPolygon(coord + dilate_epsilon * normal_vec)); 
end
