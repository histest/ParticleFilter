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
    rotmat = zeros(2, 2, particle_num);
    cosind1 = sub2ind([2, 2, particle_num], ones(particle_num, 1), ones(particle_num, 1), (1:particle_num)');
    cosind2 = sub2ind([2, 2, particle_num], ones(particle_num, 1)*2, ones(particle_num, 1)*2, (1:particle_num)');
    minussinind = sub2ind([2, 2, particle_num], ones(particle_num, 1), ones(particle_num, 1)*2, (1:particle_num)');
    sinind = sub2ind([2, 2, particle_num], ones(particle_num, 1)*2, ones(particle_num, 1), (1:particle_num)');
    
    rotmat(cosind1) = cos_thetas;
    rotmat(cosind2) = cos_thetas;
    rotmat(sinind) = sin_thetas;
    rotmat(minussinind) = -sin_thetas;
    
    %% apply noise to odometry data
    odo_noise = normrnd(repmat(odo_data', 1, particle_num), repmat(odometry_std', 1, particle_num)); 
    odo_pos = reshape(odo_noise(1:2, :), [1, 2, particle_num]);
    odo_angle = odo_noise(3, :)';
    
    odo_pos_update = sum(bsxfun(@times, rotmat, odo_pos), 2); %2x1xparticle num
    odo_pos_update = reshape(odo_pos_update, [2, particle_num])'; %particle num x 2
    
    %% update particles
    particles(:, 1:2) = particles(:, 1:2) + odo_pos_update;
    particles(:, 3) = particles(:, 3) + odo_angle; %updating angle is much more straight forward
        
    
end

