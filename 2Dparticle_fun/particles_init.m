function [ particles, weights ] = particles_init( MAP, particle_num )

    particles = zeros(particle_num, 3);
    weights = ones(particle_num, 1)./particle_num;
    par_std = [0, 0, 0];
    if (MAP.isnew)
        %distribute around zero pose with some gaussian distribution
        particles = normrnd(particles, repmat(par_std, particle_num, 1));
    else %otherwise randomly distribute the particles throughout the map
        particles(:, 1) = rand(particle_num, 1) * (MAP.xmax - MAP.xmin) + MAP.xmin;
        particles(:, 2) = rand(particle_num, 1) * (MAP.ymax - MAP.ymin) + MAP.ymin;
    end
    
end

