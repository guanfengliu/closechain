function joint_pos = fwd_kin_R(len, phi, allpoints)
% fwd_kin_R - forward kinematics of a planar, all-revolute manipulator
%---------------------------------------------------------------------
% Compute the positions of all the distal joints in a planar manipulator
% whose links are of length len(i) with link 1 pinned to the origin.
% All joints are assumed to be revolute.  This function is vectorized,
% so that passing in a matrix whos columns of angles, phi, returns the
% same number of columns of joint positions.
%
% Syntax:  joint_pos = fwd_kin_R(len, phi)   
%
% ******************** ARGUMENT DEFINITIONS ***********************
%
% len         Vector containing the lengths of the manipulator
%               starting with the base link and moving distally.
%               The number of links is num_links = length(len).
%
% phi         Matrix of dimension (num_links x num_configs). Each
%               column is interpretted as a configuration of the
%               manipulator.  The joint angles are measured in radians
%               in the ccw direction relative to the positive x-axis.  
%
% joint_pos   The ((2*num_links) x num_configs) array of joint positions,
%               working distally from the first joint not at the origin.
%               Each column has the form [x1; y1; x2; y2; ... xn; yn],
%               where (xi,yi) is the position of the end of link i and
%               also the location of joint (i+1).  Joint 1 is assumed
%               to coincide with the origin.  For a closed chain (assuming
%               the ground link is included in the data), the last two rows
%               of joint_pos should be all zeros.
%
% ******************** END ARGUMENT DEFINITIONS ***********************

% written by: Jeff Trinkle 1/30/00
%    1) Vectorized by Jeff Trinkle 2/27/00.  For a single configuration,
% the vectorized code is faster than the non-vectorized code when there
% are more than 5 links.  For a 5-link manipulator, finding 100
% configurations is about 50 times faster vectorized.  1000 configuration
% of a 10 link manipulator is computed about 65 times faster when vectorized.
%---------------------------------------------------------------------------
if nargin ~= 3
   error('   Three arguments required!!!');
end


% The last two elements of each column is the position of the tip of the
% manipulator.  For the configuration specified by the corresponding column
% of the input array, phi.
num_links = length(len);
szphi = size(phi);
num_configs = szphi(2);
if num_links ~= szphi(1)
   if num_links == num_configs
      phi = phi';
      num_configs = szphi(1);
      fprintf('\n\n<++++++++++ Warning in fwd_kin_R ++++++++++++++>');
      fprintf('\n Apparently you passed the transpose of the configuration');
      fprintf('\n array.  Transposing, crossing fingers, and continuing.');
      fprintf('\n');
   else
      error('Length of link vector not equal to number of rows in configuration array.!');
   end
end
% Be sure len is a column vector
len = reshape(len, num_links, 1);

% Allocate all the memory.  Could do this with the zeros() function, but
% I believe that this is faster.
joint_pos = [phi; phi];


% Compute all the cumulative sums of the lengths times the sines and cosines
% of the angles.  Then place the x-components in the odd colums of hte 
% output array and the y-components in the even columns.
if num_links == 1  % Avoid unwanted behavior of cumsum() when phi is a vector.
   x_pos = len(1) * cos(phi);
   y_pos = len(1) * sin(phi);
else
   % Replicate len to allow efficient vector operations.
   rep_len = repmat(len, 1, num_configs);
   x_pos = cumsum(rep_len .* cos(phi));
   y_pos = cumsum(rep_len .* sin(phi));
end

if ~allpoints
    joint_pos=[x_pos(num_links,:);y_pos(num_links,:)];
    return;
end

% Prepare for insertion of x and y position data into appropriate columns 
% of the output array.
x_rows = 1:2:2*num_links;
y_rows = 1 + x_rows;

joint_pos(x_rows, :) = x_pos;
joint_pos(y_rows, :) = y_pos;
return;