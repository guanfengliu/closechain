function out = CollisionCheck (fv1, fv2)
% Determine if two sets of triangular faces overlap

n1 = size (fv1.faces, 1);
n2 = size (fv2.faces, 1);

for i = 1:n1
    P1 = fv1.vertices(fv1.faces(i,:), :);
    min_P1_x = min(P1(:,1));
    max_P1_x = max(P1(:,1));
    min_P1_y = min(P1(:,2));
    max_P1_y = max(P1(:,2));
    for j = 1:n2
        P2 = fv2.vertices(fv2.faces(j,:), :);
        min_P2_x = min(P2(:,1));
        max_P2_x = max(P2(:,1));
        min_P2_y = min(P2(:,2));
        max_P2_y = max(P2(:,2));
        if min_P1_x > max_P2_x ||...
           max_P1_x < min_P2_x ||...
           min_P1_y > max_P2_y ||...
           max_P1_y < min_P2_y
           %% skip
        else
          if (triangle_intersection(P1,P2))
              out = true;
              return;
          end
        end
    end
end

out = false;
