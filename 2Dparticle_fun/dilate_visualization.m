%use 5 particles to visualize
figure(1)
particles = zeros(1, 3);

odo_body = odo_global2body(lidar);

plot(particles(:, 1), particles(:, 2), '.')
hold on
for i = 1:10000
    particles = odo_update(odo_body(i, :), [0.005, 0.005, 0.001], particles);
    plot(particles(:, 1), particles(:, 2), 'b.');
end