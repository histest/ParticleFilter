function [particles, weights, best_pose, MAP, xim, yim] = particle_filter( LidarData, odd_occ, odd_free, particles, weights, MAP)
    %% parameters
    persistent moving_avg;
    moving_avg_window = 20;
    update_thresh = 0.97;
    [particle_num, ~] = size(particles);
    %num_lidar_readings = 1081;
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
    ValidInd = (LidarData < 30) & (LidarData > 0.3);
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
        Y = [x_hit; y_hit; zeros(size(x_hit))];
       
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
        %x_MAP = round((x_im - MAP.xmin)./MAP.res);
        %y_MAP = round((MAP.ymax - y_im)./MAP.res);
        
        
        %[Y_mesh, X_mesh] = meshgrid(y_MAP, x_MAP);
        %ind_MAP = sub2ind([MAP.sizey, MAP.sizex], Y_mesh, X_mesh);
        %finally, grab cell values from global map
        %local_map = int8(MAP.logmap(ind_MAP));
        %define the range that i want to look for
        %x_range = MAP.res:MAP.res:0.1*MAP.res;
        %y_range = MAP.res:MAP.res:0.1*MAP.res;
        %x_range = 0;
        %y_range = 0;
        
        %calculate correlation
        if isempty(isempty(x_im) || isempty(y_im))
            corr_all(p) = 0;
        else
            %c = map_correlation(local_map, y_im, x_im, Y, x_range, y_range);
            c = map_correlation(int8(MAP.logmap + 50), MAP.xmin:MAP.res:MAP.xmax, MAP.ymin:MAP.res:MAP.ymax, Y, 0, 0);
            corr_all(p) = c;
            
            %{
            [y_cor, x_cor] = ind2sub([3,3], correction);
            %correct particle position
            x_cor = (x_cor - 2)*0.1*MAP.res;
            y_cor = (y_cor - 2)*0.1*MAP.res;
            particles(p, 1) = particles(p, 1) + x_cor;
            particles(p, 2) = particles(p, 2) + y_cor;
            %}
            
        end
    end
    
    %% update weights
    weights = weights.*corr_all;
    weights = weights./sum(weights);
    
    %% update map
    %find best particle
    [best_corr, best_p] = max(corr_all);
    
    if isempty(moving_avg)
        moving_avg = ones(moving_avg_window,1) * best_corr;
    end
    
    fprintf('best_corr: %.3f ', best_corr)
    best_pose = particles(best_p, :);
    
    if (1)% best_corr >= mean(moving_avg)*update_thresh && best_corr > 70000 %only update map when there's enough correlation
        [MAP, xim, yim] = MAP_update(best_pose, LidarData, odd_occ, odd_free, MAP);
    else
        fprintf('no update ')
        xim = ceil(best_pose(1) - MAP.xmin)./MAP.res;
        yim = ceil(best_pose(2) - MAP.ymin)./MAP.res;
    end
    
    moving_avg(1:moving_avg_window-1) = moving_avg(2:moving_avg_window);
    moving_avg(moving_avg_window) = best_corr;
    
    
    
    %% visualize for debugging
    
    
   %% resample
   Neff = 1/sum(weights.^2);
   if Neff < 0.3*particle_num
    [particles, weights] = resample_particle(particles, weights);
    fprintf('resample')
   end
end