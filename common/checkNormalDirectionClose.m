        %% check if diffVec has <90 degree from the normal_vec
        % @normal_vec is a unit normal vector
        % @diff_vec might not be a unit vector
function positive = checkNormalDirectionClose(normal_vec, diff_vec, half_plane_epsilon)
   val = normal_vec' * (diff_vec/norm(diff_vec));
   if val > half_plane_epsilon
      positive = true;
   else
       positive = false;
   end
end
       