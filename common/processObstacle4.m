function [mpData] = processObstacle4(mpData)
% mpData.conf.ptObst, 
% mpData.conf.dilate_epsilon
% mpData.conf.numVertex
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for simplificy, for PRM, we only consider simplified obstacles which are
% generated from a point

% @mpData.conf.numObst -- number of obstcle
% @mpData.conf.ptObst(i).coord =[X;Y]  are the coordinate of ith point obstacle

% @mpData.conf.dilate_epsilon is the length to grow the pointer obstacle along a
% direction to form a vertex of the resulting polygon

%the following is for compatible with Gen2 and Gen3 random sampling
%algorithm functions
 mpData.linklength = mpData.conf.linkLengthVec;
 mpData.thickNess = mpData.conf.thickNess;
 mpData.enable_bound = mpData.conf.enable_bound;
 mpData.nonbsamples = mpData.conf.nonbsamples;  %%mpData.nobst_samples
 
[CollisionVarCritRadiusL, CollisionVarCritRadiusR] = calCritRadius(mpData.linklength);
mpData.CollisionVarCritRadiusL = CollisionVarCritRadiusL;  %% set of critical circles counting from left anchor
mpData.CollisionVarCritRadiusR = CollisionVarCritRadiusR;  %% set of critical circles counting from right anchor

angles=-3*pi/4:2*pi/mpData.conf.numVertex:3*pi/4;
angles=angles(1:mpData.conf.numVertex);
normal_vec = [cos(angles);sin(angles)];


%mpData.obstacle.numObst = obst.numObst;
tic;
mpData.obstacle.fv = [];
mpData.obstacle.fv.vertices = [];
mpData.obstacle.fv.faces = [];
mpData.obstacle.coord = [];
mpData.obstacle.numObst = mpData.conf.numObst;
mpData.num_links = length(mpData.conf.linkLengthVec);
for i=1:mpData.conf.numObst,
    %% the vertices of coord have to be arranged counter clockwise
    %% around obstacles
    mpData.obstacle.center{i}= mpData.conf.ptObst(i).coord; 
    mpData.obstacle.obstDesc{i} = mpData.obstacle.center{i} + normal_vec * mpData.conf.dilate_epsilon;
    mpData.obstacle.coord = [mpData.obstacle.coord, mpData.obstacle.obstDesc{i}];
    mpData.obstacle.fv = appendFV(mpData.obstacle.fv, TriangPolygon(mpData.obstacle.obstDesc{i}));
end
mpData.narrow_faces = [];
mpData.narrow_vertices = [];
mpData.num_narrow_pairs = 0;
[mpData]= findNarrowFaceVertexPairsClose(mpData);
disp(['preprocess time ' num2str(toc)]); 
mpData.test_list=[];  % no link-link collision checking
mpData.base=[0,0]';  % robot base point assume to be origin

mpData.max_nodes = 1e5;
mpData.position = zeros(mpData.num_links, 2, mpData.max_nodes, 'single');
mpData.nVertices = 0;
mpData.total_samples = [];
end