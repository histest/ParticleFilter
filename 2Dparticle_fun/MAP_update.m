function MAP = MAP_update(pose, LidarData, odd_occ, odd_free, MAP)
    %MAP_UPDATE1 Summary of this function goes here
    %   Detailed explanation goes here
    log_odd_max = 127; 
    log_odd_min = -127;
    %    
    scan_angle = -135:0.25:135;
    scan_angle = scan_angle/180*pi;
    
    px = pose(1);
    py = pose(2);
    p_angle = pose(3);
    %condition lidar data
    ValidInd = (LidarData < 30) & (LidarData > 0.1);
    LidarData = LidarData(ValidInd);
    scan_angle = scan_angle(ValidInd);
    %
    phi = scan_angle + p_angle;
    %calculate end points of the ray
    x_end = LidarData .* cos(phi) + px;
    y_end = LidarData .* sin(phi) + py;
    ValidX = (x_end >= MAP.xmin) & (x_end <= MAP.xmax);
    ValidY = (y_end >= MAP.ymin) & (y_end <= MAP.ymax);
    ValidXY = ValidX & ValidY;
    x_end = x_end(ValidXY);
    y_end = y_end(ValidXY);
    % put the points on grid
    px_grid = ceil((px - MAP.xmin) ./ MAP.res);
    py_grid = ceil((py - MAP.ymin) ./ MAP.res);
    x_end_grid = ceil((x_end - MAP.xmin) ./ MAP.res);
    y_end_grid = ceil((y_end - MAP.ymin) ./MAP.res);
    %get empty cells
    [x_empty, y_empty] = getMapCellsFromRay(px_grid, py_grid, x_end_grid, y_end_grid);
    %update map likelihood
    IndEmpty = sub2ind([MAP.sizex, MAP.sizey], x_empty, y_empty);
    IndFull = sub2ind([MAP.sizex, MAP.sizey], x_end_grid, y_end_grid);
    
    EmptyXFull = intersect(IndEmpty, IndFull);
    IndEmpty = setdiff(IndEmpty, EmptyXFull);
    
    MAP.logmap(IndEmpty) = MAP.logmap(IndEmpty) + odd_free;
    MAP.logmap(IndFull) = MAP.logmap(IndFull) + odd_occ;
    %saturate odd
    IndBig = MAP.logmap > log_odd_max;
    IndSmall = MAP.logmap < log_odd_min;
    MAP.logmap(IndBig) = log_odd_max;
    MAP.logmap(IndSmall) = log_odd_min;
    %update map
    MAP.map = int8(MAP.logmap > 10);
end

