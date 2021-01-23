function [output_angle] = closeLoopPseudo(linklength, startPt, endPt, initAngle, dvEpsilon, maxSteps)
%% this is a utility function that for an open chain with linklength vector
% and an initial angle, we use pseudo Jacobian inverse to close the gap
% between current end point (determined by startPt, initAngle and linklength)
% and the target end point endPt
% Inputs: @linklength,  the link length vector of the chain
% @startPt, the base point of the chain
% @endPt, the target point for the tip of the chain
%@initAngle, the initial angle of the chain (assumed to be open and its tip
% has a gap from endPt)
% Ouput: @angle, the joint angle at which the chain close the gap to make
% contact with endPt
  output_angle = [];
  DOF = length(linklength);
  if DOF < 2
      fprintf("the DOF of the chain is less than 2, can not close loop using pseudo inverse algorithm");
      return;
  end
  
  dv = endPt - startPt;
  % jacobian matrix
  angle = initAngle;
  output_angle = [output_angle, angle];
  J = [-linklength'.* sin(angle'); linklength'.* cos(angle')];
  numSteps = 0;
  while norm(dv) > dvEpsilon && numSteps < maxSteps
      dtheta = J' * inv(J * J') * dv;
      angle = angle + dtheta;
      output_angle = [output_angle, angle];
      J = [-linklength'.* sin(angle'); linklength'.* cos(angle')];
      newStartPt = startPt + [sum(linklength' .* cos(angle')); sum(linklength' .* sin(angle'))];
      dv = endPt - newStartPt;
      numSteps = numSteps + 1;
  end
  if numSteps >= maxSteps
      output_angle = [];  %% return empty
  end
end