function odo_body = odo_global2body( lidar_cell )

    num_steps = numel(lidar_cell);

    odo_body = zeros(num_steps-1, 3);
    
    for i = 1:(num_steps-1)
        cur_theta = lidar_cell{i}.pose(3);
        odo_global = lidar_cell{i+1}.pose - lidar_cell{i}.pose;
        rotmat = [cos(cur_theta), sin(cur_theta); -sin(cur_theta), cos(cur_theta)];
        odo_body(i, 1:2) = (rotmat*[odo_global(1); odo_global(2)])';
        odo_body(i, 3) = odo_global(3);
    end

end

