function link_pos = generate_link_position_close(temp_world_ang, mpData)
            % generates position and orientation of the redundant
            % manipulator model
            link_pos = zeros(mpData.num_links, 2);
            % temp_world_ang = cumsum( node );
            % Compute the link end positions
            link_pos(:, 1) = [mpData.base(1); cumsum(mpData.conf.linkLengthVec(1:end-1)'.*cos(temp_world_ang(1:end-1))) + mpData.base(1)];
            link_pos(:, 2) = [mpData.base(2); cumsum(mpData.conf.linkLengthVec(1:end-1)'.*sin(temp_world_ang(1:end-1))) + mpData.base(2)];
        end