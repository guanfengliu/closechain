function fv = TriangPolygon (coord_input)
%% Make a simple polygon and then triangulate it
pgon=polyshape(coord_input'); 
T=triangulation(pgon);
fv.vertices = T.Points;
fv.faces = T.ConnectivityList; 