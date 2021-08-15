 %% calculate the minimal distance between qq and the line segment pp1->pp2
 function [dist, closestPt] = computeDistVertex2EdgeClose(qq, pp1, pp2, offset)
            dv = pp2 - pp1;
            dq = pp1 - qq;
            t = - (dq' * dv)/((norm(dv))^2);
            if t >= 0 && t <= 1
                dist = abs(dv(1) * dq(2) - dv(2) * dq(1))/norm(dv);
                closestPt = pp1 + t * dv;
            else
                if norm(qq-pp1) < norm(qq-pp2)
                  dist = norm(qq-pp1);
                  closestPt = pp1;
                else
                  dist = norm(qq-pp2);
                  closestPt = pp2;
                end
               % dist = min(norm(qq - pp1), norm(qq-pp2));
            end
            % subtract the half of the link wdith
            dist = dist - offset;
end