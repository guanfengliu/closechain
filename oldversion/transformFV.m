function out = transformFV (fv, theta, t)
%% Rotate and translate a patch data structure (face and vertex)
%% this function is mainly for tranforming a rigid polygonal collision model by a rigid motion
out.faces = fv.faces;

c = cos(theta);
s = sin(theta);

out.vertices = bsxfun(@plus, fv.vertices*[c s; -s c], t);
