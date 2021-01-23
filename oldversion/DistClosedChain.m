function out = DistClosedChain (x1, x2)
%% Compute the distance between two configurations of a closed chain
% Note we assume all angular coordinates are between 0 and 360
% x1,x2 are all row vectors; the last angle is always pi, the link fixed to
% the ground
useRelative=false;

if useRelative
    sz1=size(x2,1);
    sz2=size(x2,2);
    y1=zeros(sz1,1);
    y2=zeros(sz1,sz2);
    y1(1)=mod(x1(1),2*pi);
    y2(1,:)=mod(x2(1,:),2*pi);
    y1(2:sz1)=mod(x1(2:sz1)-x1(1:sz1-1),2*pi);
    y2(2:sz1,:)=mod(x2(2:sz1,:)-x2(1:sz1-1,:),2*pi);
else
    y1=mod(x1,2*pi);
    y2=mod(x2,2*pi);
end

e = abs(bsxfun(@minus, y2, y1));
out = sum(min(e, 2*pi-e));
