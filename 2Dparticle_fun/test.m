addpath ../mex
load ../train_lidar2.mat

%init Map, since we know nothing about the map, the odds of a cell being
%occupied is 50/50, ending up in log odd of 0
MAP.res = 0.1;%meters
MAP.xmin = -100;%meters
MAP.ymin = -100;
MAP.xmax = 100;
MAP.ymax = 100;
MAP.sizex = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1);
MAP.sizey = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);
MAP.logmap = zeros(MAP.sizex, MAP.sizey);
MAP.map = zeros(MAP.sizex, MAP.sizey, 'int8');
MAP.isnew = 1;

%init parameters (precision of lidar)
p_z1_given_m1 = 0.6; %tunable, true positive
p_z0_given_m1 = 1 - p_z1_given_m1;
p_z1_given_m0 = 0.15; %tunable, false positive
p_z0_given_m0 = 1 - p_z1_given_m0;

%calculate the log odd
odd_occ = log2(p_z1_given_m1/p_z1_given_m0);
odd_free = log2(p_z0_given_m1/p_z0_given_m0);

%initialize odometry noise to inject into update
odometry_cov = [0.005, 0.005 ,0.008];

%init particles if map is new, initialize all the particles all at 0, 0 
%otherwise initialize randomly or according to prior distribution if no 
particle_num = 100; %use 100 particles to begin
[particles, weights] = particles_init(MAP, particle_num);
particles = odo_update([0, 0, 0], [0, 0, 0], particles);

%before running particle filter, first construct the map with the first
%lidar reading
LidarData = double(lidar{1}.scan);
MAP = MAP_update([0, 0, 0], LidarData, 10*odd_occ, 10*odd_free, MAP);
odometry = odo_global2body(lidar);


%% plots convert particles to map
figure(1);
pvisualx = (particles(:, 1) - MAP.xmin) ./ MAP.res;
pvisualy = (particles(:, 2) - MAP.ymin) ./ MAP.res;
h_particles = plot(pvisualx, pvisualy, 'r.');
hold on
wallind = find(MAP.map);
[xwall, ywall] = ind2sub([MAP.sizex, MAP.sizey], wallind);
h_wall = plot(xwall, ywall, 'b.');

particle_hist = zeros(numel(lidar),3);
%%

for i = 1:numel(lidar)
    %update particles according to odometry data
    LidarData = double(lidar{i}.scan);
    particles = odo_update(odometry(i, :), odometry_cov, particles);
    
    %{
    figure(2)
    plot(particles(:,3))
    odometry(i, 3)
    %}
    
    %given measurements, update particles, weights and global map
    [particles, weights, pose, MAP] = particle_filter(LidarData, odd_occ, odd_free, particles, weights, MAP);
    particle_hist(i, :) = pose;

    pvisualx = (particles(:, 1) - MAP.xmin) ./ MAP.res;
    pvisualy = (particles(:, 2) - MAP.ymin) ./ MAP.res;
    wallind = find(MAP.map);
    [xwall, ywall] = ind2sub([MAP.sizex, MAP.sizey], wallind);
    
    set(h_particles, 'xdata', pvisualx, 'ydata', pvisualy);
    set(h_wall, 'xdata', xwall, 'ydata', ywall);
    fprintf('%d\n', i)
    pause(0.001)
end

%%

