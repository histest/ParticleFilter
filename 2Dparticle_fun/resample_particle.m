function [ particles, weights ] = resample_particle( particles, weights )
    %RESAMPLE Summary of this function goes here
    %   Detailed explanation goes here
    [num_p, ~] = size(weights);
    weightCDF = cumsum(weights);
    %wheel sampling
    step = 1/num_p;
    original = linspace(0, 1-step, num_p);
    u_randval = rand*step+original;
    ind_resample = BinSearch_interval(weightCDF, u_randval);   
    
    particles = particles(ind_resample, :);
    weights = ones(num_p, 1)./num_p;
end

