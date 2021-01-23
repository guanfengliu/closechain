function [fv, fvList] = triangleChain(ankerPt,linklength,thickness,cspace)
% Simulate a simple n link revolute chain 
% @ankerPt is the base of the robot
% @linklength is the vector of link lengths
% here cspace(i) are absolute joint angles w.r.t. horizontal axis
% l1 left end ankered at origin, and right end determined by l1 * (cos (cspace(1)), sin (cspace(1)))  

DOF=length(linklength);
fvList = cell(DOF, 1);

% we keep track of anker point for appending together all links
old=false; % if there exists a previous link already

for i = 1:DOF
    l=linklength(i);
    h=thickness(i)/2.0;
    link=boxFV(0,l,-h,h);
    fv=transformFV(link,cspace(i),ankerPt);
    fvList{i} = fv;
    if old
         fv = appendFV(fv,oldfv);
    end 
    ankerPt=ankerPt + l*[cos(cspace(i)),sin(cspace(i))];
    old=true;
    oldfv=fv;
end

%fv = transformFV(fv, cspace(end), [0 0]);
