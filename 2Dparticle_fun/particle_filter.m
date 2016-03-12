function [particles, weights, best_pose, MAP] = particle_filter( LidarData, odd_occ, odd_free, particles, weights, MAP)
    %% parameters
    
    [particle_num, ~] = size(particles);
    num_lidar_readings = 1081;
    scan_angle = -135:0.25:135;
    scan_angle = scan_angle * pi / 180;
    
    %local map parameters
    local_min = -30;
    local_max = 30;
    
    %temporary storage for correlation
    corr_all = zeros(particle_num, 1);
    
    %% transform lidar data from local frame to global frame and calculate correlation
    
    %if distance is greater than 30 that means the lidar didn't hit
    %anything
    %find valid lidar readings
    ValidInd = (LidarData < 30) & (LidarData > 0.1);
    LidarData_pf = LidarData(ValidInd);
    scan_angle = scan_angle(ValidInd);
    
    %for each particle
    for p = 1:particle_num
        %define particle values
        px = particles(p, 1);
        py = particles(p, 2);
        p_angle = particles(p, 3);
        
        
        
        %rotate local map to global map
        phi = scan_angle + p_angle;
        
        %find locations which lidar hit
        x_hit = LidarData_pf .* cos(phi) + px;
        y_hit = LidarData_pf .* sin(phi) + py;
        
        %put it into the format that the correlation function likes
        Y = [y_hit; x_hit; zeros(size(x_hit))];
        
        %make local map from global map
        %first grab the range
        local_xmin = px+local_min;
        local_xmax = px+local_max;
        local_ymin = py+local_min;
        local_ymax = py+local_max;
        %calculate cell coordinates
        x_im = local_xmin : MAP.res : local_xmax;
        y_im = local_ymin : MAP.res : local_ymax;
        %condition away coordinates that are outside of map
        ValidX = (x_im >= MAP.xmin) & (x_im <= MAP.xmax);
        ValidY = (y_im >= MAP.ymin) & (y_im <= MAP.ymax);
        x_im = x_im(ValidX);
        y_im = y_im(ValidY);
        
        %xis = (ceil((x_hit - px - local_min)./ MAP.res ));
        %yis = (ceil((y_hit - py - local_min)./ MAP.res ));
        
        %then find the indicies to take from global map
        x_MAP = ceil((x_im - MAP.xmin)./MAP.res);
        y_MAP = ceil((y_im - MAP.ymin)./MAP.res);
        
        
        [X_mesh, Y_mesh] = meshgrid(x_MAP, y_MAP);
        ind_MAP = sub2ind([MAP.sizex, MAP.sizey], X_mesh, Y_mesh);
        %finally, grab cell values from global map
        local_map = MAP.map(ind_MAP);
        %define the range that i want to look for
        x_range = 0;
        y_range = 0;
        
        %calculate correlation
        if (isempty(local_map) || isempty(x_im) || isempty(y_im))
            corr_all(p) = 0;
        else
            c = map_correlation(local_map, x_im, y_im, Y, x_range, y_range);
            corr_all(p) = sum(sum(c));
        end
    end
    
    
    %% update weights
    weights = weights.*corr_all;
    weights = weights./sum(weights);
    
    %% update map
    %find best particle
    [~, best_p] = max(weights);
    
    best_pose = particles(best_p, :);
    MAP = MAP_update(best_pose, LidarData, odd_occ, odd_free, MAP);
    %% visualize for debugging
    
    
   %% resample
   if (max(weights) > 0.5)
    [particles, weights] = resample_particle(particles, weights);
   end
end