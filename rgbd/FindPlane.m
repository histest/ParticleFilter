function points = FindPlane( Depth )
    %% parameters
    DEPTH_MAX = 4500; % recommended max range
DEPTH_MIN = 400;  % recommended min range
    episilonRANSAC = 0.05;
    pitch = Depth.head_angles(2);
    h = 0.93 + 0.395 + 0.085; % in meters
    R = [1, 0, 0;
         0, cos(pitch), -sin(pitch);
         0, sin(pitch), cos(pitch)];
    GGN = [0, 1, 0]'; %global ground normal (IMU)
    CGN = R*GGN; %camera frame ground normal (IMU)
    
    a = CGN(1);
    b = CGN(2);
    c = CGN(3);
    d = -h;
    
    D = Depth.depth;
    D(D(:) <= DEPTH_MIN) = 0;
    D(D(:) >= DEPTH_MAX) = 0;  
    D = medfilt2(D,[3 3]);      % if you want to filter noise
    
    %% shoot out ray from camera
    fc = [364.4574, 364.5428];
    cc = [259.4425; 202.4871];
    
    invCalib = [1/fc(2), 0, -cc(2)/fc(2);
                0, 1/fc(1), -cc(1)/fc(1);
                0,  0,     1];
            
    Calib = [fc(1), 0, cc(1);
            0, fc(2), cc(2);
            0, 0, 1];
    [imX, imY] = meshgrid(1:512, 1:424);
    
    Ray_pixel = [imY(:)'; imX(:)'; ones(1, 512*424)];
    Ray_image = invCalib * Ray_pixel;
    Ray_image = bsxfun(@times, Ray_image, 1./sqrt(sum(Ray_image.^2, 1)));
    points_camera = bsxfun(@times, Ray_image', D(:)/1000);
    temp = points_camera(:, 1);
    points_camera(:, 1) = points_camera(:, 2);
    points_camera(:, 2) = temp;
    
    points_to_choose = points_camera;
    points_to_choose(all(points_to_choose == 0, 2), :) = [];
    
    
    [sample_num, ~] = size(points_to_choose);
    
    %% do ransac to find the best plane
    
    best_inliers = 0;
    inliers_ind = 0;
    
    for i = 1:1000
        %pick 3 random points
        picks = randsample(sample_num, 3);
        PointsPicked = points_to_choose(picks, :);
        planevec = PointsPicked\[1;1;1];
        
        
        if dot(planevec, CGN) < 0.95*norm(planevec) 
            continue; %move on if it's not ground plane
        end
        
        
        DisFromPlane = (abs(points_to_choose*planevec-1))/norm(planevec);
        inliers = (DisFromPlane < episilonRANSAC);
        num_inliers = sum(inliers);
        
        if num_inliers > best_inliers
            inliers_ind = inliers;
            bestvec = planevec;
            best_inliers = num_inliers;
        end
    end        
    points = points_to_choose(inliers_ind, :);
    %{
    twoD = Calib*points';
    twoD = bsxfun(@times, twoD, 1./twoD(3, :));
    twoD = round(twoD);
    
    pic = zeros(424, 512);
    drop = sub2ind([424, 512], twoD(2, :), twoD(1, :));
    pic(drop) = Depth.depth(drop);
    imagesc(pic);
    %}
    
    
end

