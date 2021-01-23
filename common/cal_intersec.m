function p5 = cal_intersec(p1,p2,p3,p4)
%% given two lines p1p2; p4p3,
%% calculate their intersection points
epsilon1 = 1e-5;
epsilon2 = 1e-2;
p5 = [];
%% Line AB represented as a1x + b1y = c1 
a1 = p2(2) - p1(2); 
b1 = p1(1) - p2(1); 
c1 = a1 * p1(1) + b1 * p1(2); 
  
%%Line CD represented as a2x + b2y = c2 
a2 = p4(2) - p3(2); 
b2 = p3(1) - p4(1); 
c2 = a2 * p3(1)+ b2 * p3(2); 

%% compute determinant
determinant = a1 * b2 - a2 * b1; 
if abs(determinant) < 1e-5
  %% The lines are parallel. This is simplified 
  %% returning a point in the center of (p2,p3) offset by an epsilong along p1p2
  p5 = (p2 + p3)/2.0;
  p5 = p5 + epsilon2 * (p2 - p1) / norm(p2 - p1);
else 
  p5(1,1) = (b2 * c1 - b1 * c2) / determinant; 
  p5(2,1) = (a1 * c2 - a2 * c1) / determinant; 
end 