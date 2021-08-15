function [mpData] = update_link_position_close(ind, mpData)
            % Update info about the manipulator
            world_ang = mpData.total_samples(:, ind);   %cumsum( this.tree.angle(:, ind) );
            % Compute the link end positions
            mpData.position(:, 1, ind) = [mpData.base(1); cumsum( mpData.conf.linkLengthVec(1:end-1)'.*cos(world_ang(1:end-1))) + mpData.base(1)];
            mpData.position(:, 2, ind) = [mpData.base(2); cumsum( mpData.conf.linkLengthVec(1:end-1)'.*sin(world_ang(1:end-1))) + mpData.base(2)];
        end