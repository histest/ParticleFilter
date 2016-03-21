load DEPTH_3.mat
load cameraParam/exParams.mat
load ../2Dparticle_fun/map3result.mat
load ../train_lidar3.mat
%%
dummyimg = uint8(zeros(1080, 1920, 3));
[mapc, mapr] = size(MAP.map);
dummymap = uint8(zeros(mapc, mapr, 3));
TT = zeros(numel(lidar),1);
    for t = 1:numel(lidar)
        TT(t) = lidar{t}.t;
    end
for i = 1:numel(DEPTH)    
    if mod(i,300)==1
        rgb_file = sprintf('RGB_3_%d.mat',int32(i/300)+1);
        disp(strcat('loading ',rgb_file,'....'))
        load(rgb_file)
        % for example,
        % RGB_3_1.mat has   1 to 300
        % RGB_3_2.mat has 301 to 600
        % ...
        % RGB_3_4.mat has 901 to 996
    end
    
    pointsD = FindPlane(DEPTH{i});
    pointsRGB = R*pointsD';
    pointsRGB = bsxfun(@plus, pointsRGB, T/1000);
    
    %project back
    fc = [ 1049.331752604831308 ; 1051.318476285322504 ];
    cc = [ 956.910516428015740 ; 533.452032441484675 ];
    CalibRGB = [fc(1), 0, cc(1);
        0, fc(2), cc(2);
        0,    0,      1];
    
    RGB_pixel = CalibRGB*pointsRGB;
    RGB_pixel = round(bsxfun(@times, RGB_pixel, 1./RGB_pixel(3,:)));
    
    
    c_sub = RGB_pixel(2,:);
    r_sub = RGB_pixel(1,:);
    valid_sub = c_sub < 1080 & c_sub > 1 & r_sub < 1920 & r_sub > 1;
    
    c_sub = c_sub(valid_sub);
    r_sub = r_sub(valid_sub);
    
    gan = ones(1, length(c_sub));
    onetwothree = [gan, gan*2, gan*3];
    
    r_ind = mod(i,300); if r_ind == 0, r_ind = 300; end
    sprayinds = sub2ind([1080, 1920, 3], repmat(c_sub,1, 3), repmat(r_sub, 1, 3), onetwothree);
    dummyimg(sprayinds) = RGB{r_ind}.image(sprayinds);
    
    %{
    figure(2)
    imshow(dummyimg)
    %}
    %% okay then find the closest pose on slam timestamp
    
    t_now = DEPTH{i}.t;
    [~, indt] = min(abs(t_now - TT));
    pose_t = particle_hist(indt, :);
    
    %% transformation from camera to world frame
    pitch = DEPTH{i}.head_angles(2);
    bearing = pose_t(3);
    R_pitch = [1, 0, 0;
               0, cos(pitch), sin(pitch);
               0, -sin(pitch), cos(pitch)];
    R_bearing = [cos(bearing), 0, sin(bearing);
                0, 1, 0;
                -sin(bearing), 0, cos(bearing)];
    R_def = [0, 0, 1;
            -1, 0, 0;
            0, -1, 0];
    R_ctow = R_def*R_bearing*R_pitch;
    
    pointsW = R_ctow*pointsD';
    
    Xspray = ceil(((pointsW(1,:) + pose_t(1))-MAP.xmin)/MAP.res);
    Yspray = ceil(((pointsW(2,:) + pose_t(2))-MAP.ymin)/MAP.res);
    
    maprgb = [ones(1, sum(valid_sub)), 2*ones(1, sum(valid_sub)), 3*ones(1, sum(valid_sub))];
    
    indspray = sub2ind([mapc, mapr, 3], repmat(Xspray(valid_sub), 1, 3), repmat(Yspray(valid_sub), 1, 3), maprgb);
    dummymap(indspray) = RGB{r_ind}.image(sprayinds);
    
    %{
    figure(3)
    plot(xwall, ywall, '.')
    hold on
%}
    
end