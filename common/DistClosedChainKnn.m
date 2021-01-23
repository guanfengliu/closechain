function out = DistClosedChainKnn (x1, x2)
%% Compute the distance between two configurations of a closed chain
% Note we assume all angular coordinates are between 0 and 360
% x1 is 1 by n row vector, and x2 is k by n matrix all row vectors; the last angle is always pi, the link fixed to
% the ground
dim=length(x1);
y1=mod(x1(:,3:dim-1),2*pi);  %% only measure 3:dim
y2=mod(x2(:,3:dim-1),2*pi);

e = abs(bsxfun(@minus, y2, y1));
re = min(e, 2*pi-e);
out = vecnorm(re')'; %sum(min(e, 2*pi-e))
