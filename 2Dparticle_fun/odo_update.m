function particles = odo_update( odo_data, odometry_std, particles )
    
    %the passed in odo_data is in global frame, change it to local frame
    %for each particle
    
    %in this 2D slam case, odo_data is [x_delta, y_delta, theta_delta]
    %make rotation matrix from current pose
    
    [particle_num, ~] = size(particles);
    
    thetas = particles(:,3);
    cos_thetas = cos(thetas);
    sin_thetas = sin(thetas);
    
    
    
    %% build rotation matrices
    odo_x = cos_thetas.*odo_data(1) - sin_thetas.*odo_data(2);
    odo_y = sin_thetas.*odo_data(1) + cos_thetas.*odo_data(2);
    
    odo_update = [odo_x, odo_y, repmat(odo_data(3), particle_num, 1)];
    
    %% apply noise
    odo_update = normrnd(odo_update, repmat(odometry_std, particle_num, 1)); 
    
    %% update particles
    particles = particles + odo_update;        
end

