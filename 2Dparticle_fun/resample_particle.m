function [ particles, weights ] = resample_particle( particles, weights )
    %RESAMPLE Summary of this function goes here
    %   Detailed explanation goes here
    [num_p, ~] = size(weights);
    weightCDF = cumsum(weights);
    u_randval = rand(num_p, 1);
    ind_resample = BinSearch_interval(weightCDF, u_randval);   
    
    particles = particles(ind_resample, :);
    weights = ones(num_p, 1)./num_p;
end

