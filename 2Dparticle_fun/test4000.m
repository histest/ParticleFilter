addpath ../mex
load ../train_lidar2.mat
load map3500.mat

%map loaded
starting = 3500;

%init parameters (precision of lidar)
p_z1_given_m1 = 0.7; %tunable, true positive
p_z0_given_m1 = 1 - p_z1_given_m1;
p_z1_given_m0 = 0.05; %tunable, false positive
p_z0_given_m0 = 1 - p_z1_given_m0;

%calculate the log odd
odd_occ = log10(p_z1_given_m1/p_z1_given_m0);
odd_free =log10(p_z0_given_m1/p_z0_given_m0);

%initialize odometry noise to inject into update
odometry_cov = [1e-5, 1e-5 ,6e-5];

%init particles if map is new, initialize all the particles all at 0, 0 
%otherwise initialize randomly or according to prior distribution if no 
%particle_num = 30; %use 100 particles to begin
%[particles, weights] = particles_init(MAP, particle_num);
%particles = particles(1:particle_num, :);

%before running particle filter, first construct the map with the first
%lidar reading
LidarData = double(lidar{starting}.scan);
%MAP = MAP_update([0, 0, 0], LidarData, 200*odd_occ, 200*odd_free, MAP);
odometry = odo_global2body(lidar);


%% plots convert particles to map
figure(1);
pvisualx = (particles(:, 1) - MAP.xmin) ./ MAP.res;
pvisualy = (particles(:, 2) - MAP.ymin) ./ MAP.res;
h_particles = plot(pvisualx, pvisualy, 'r.');
hold on
wallind = find(MAP.map);
[xwall, ywall] = ind2sub([MAP.sizex, MAP.sizey], wallind);
h_hist = plot(0, 0)
h_wall = plot(xwall, ywall, 'b.');
h_update = plot(0, 0, 'y.');
h_quiver = quiver(pvisualx, pvisualy, pvisualx + 20*cos(particles(:,3)), pvisualy+20*sin(particles(:,3)));
%%

for i = starting:numel(lidar)-1
    %update particles according to odometry data
    LidarData = double(lidar{i}.scan);
    particles = odo_update([odometry(i,2), odometry(i, 1), odometry(i, 3)], odometry_cov, particles);
    %{
    if i == 4500
        save map4500.mat
    end
    
    figure(2)
    plot(particles(:,3))
    odometry(i, 3)
    %}
    %given measurements, update particles, weights and global map
    [particles, weights, pose, MAP, xim, yim] = particle_filter(LidarData, odd_occ, odd_free, particles, weights, MAP);
    particle_hist(i, :) = pose;

    pvisualx = (particles(:, 1) - MAP.xmin) ./ MAP.res;
    pvisualy = (particles(:, 2) - MAP.ymin) ./ MAP.res;
    wallind = find(MAP.map);
    [xwall, ywall] = ind2sub([MAP.sizex, MAP.sizey], wallind);
    
    set(h_particles, 'xdata', pvisualx, 'ydata', pvisualy);
    set(h_wall, 'xdata', xwall, 'ydata', ywall);
    set(h_update, 'xdata', xim, 'ydata', yim);
    set(h_hist, 'xdata', (particle_hist(1:i, 1)-MAP.xmin)./MAP.res, 'ydata', (particle_hist(1:i, 1)-MAP.ymin)./MAP.res)
    set(h_quiver,'xdata', pvisualx, 'ydata', pvisualy, 'udata', pvisualx + 2*cos(particles(:,3)), 'vdata', pvisualy + 2*sin(particles(:,3)))
    
    fprintf('%d\n', i)
    pause(0.001)
end