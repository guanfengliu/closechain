function out = DistClosedChain (x1, x2)
%% Compute the distance between two configurations of a closed chain
% Note we assume all angular coordinates are between 0 and 360
% x1,x2 are all row vectors; the last angle is always pi, the link fixed to
% the ground
dim=length(x1);
y1=mod(x1(3:dim-1,:),2*pi);  %% only measure 3:dim
y2=mod(x2(3:dim-1,:),2*pi);

e = abs(bsxfun(@minus, y2, y1));
out = vecnorm(min(e, 2*pi-e)); %sum(min(e, 2*pi-e))
