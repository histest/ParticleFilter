clear all;
close all;

addpath('./cameraParam');

load DEPTH_3.mat
run('IRcamera_Calib_Results')

DEPTH_MAX = 4500; % recommended max range
DEPTH_MIN = 400;  % recommended min range

for k=[1:numel(DEPTH)]
    
    % RGB data is chopped into multiple files because of its size issue
    % Next RGB mat file needs to be loaded every 300 frames
    if mod(k,300)==1
        rgb_file = sprintf('RGB_3_%d.mat',int32(k/300)+1);
        disp(strcat('loading ',rgb_file,'....'))
        load(rgb_file)
        % for example, 
        % RGB_3_1.mat has   1 to 300 
        % RGB_3_2.mat has 301 to 600 
        % ...
        % RGB_3_4.mat has 901 to 996
    end
    
    D = DEPTH{k}.depth;
    D(D(:) <= DEPTH_MIN) = 0;
    D(D(:) >= DEPTH_MAX) = 0;  
    D = medfilt2(D,[3 3]);      % if you want to filter noise
    
    % correct the index
    r_ind = mod(k,300); if r_ind == 0, r_ind = 300; end
    R = RGB{r_ind}.image;

    figure(1), imagesc(D)
    figure(2), imshow(R)
   
    pause(0.2); 
end
