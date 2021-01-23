function [fv] = closedchainthick (linklength,cspace,thickness)
% Simulate a simple n link revolute chain 
% here cspace(i) are absolute joint angles w.r.t. horizontal axis
% l1 left end ankered at origin, and right end determined by l1 * (cos (cspace(1)), sin (cspace(1)))  
% the last link, link(DOF)is always horizontal, and more over it fold back until overlaping with the origin, so 
% cspace(DOF)= pi

%%% parameter to see if we need to check the base link in collision
%%% checking
checkBaseLink=false;

DOF=size(linklength,2);

%l = linklength(DOF);
%h = 0.5;

%link = boxFV(0,l,-h,h);
%fv=transformFV(link,cspace(DOF),[0,0]);
%fv = link;


% we keep track of anker point for appending together all links
ankerPt=[0,0];
old=false; % if there exists a previous link already
if checkBaseLink
    endIndex=DOF;
else
    endIndex=DOF-1;
end

for i = 1:endIndex
    l=linklength(i);
    h=thickness;
    link=boxFV(0,l,-h,h);
    fv=transformFV(link,cspace(i),ankerPt);
    if old
         fv = appendFV(fv,oldfv);
    end 
    ankerPt=ankerPt + l*[cos(cspace(i)),sin(cspace(i))];
    old=true;
    oldfv=fv;
end

%fv = transformFV(fv, cspace(end), [0 0]);
