odo_check = odo_global2body(lidar);

par_hist = zeros(2001,3);
particle = [0, 0, pi/2];

for i = 1:2000
    odo_now = odo_check(i+2000, :);
    odo_now(3) = 0;
    particle = odo_update( odo_now, [0, 0, 0], particle);
    par_hist(i, :) = particle;
end
    