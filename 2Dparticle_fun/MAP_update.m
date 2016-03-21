function [MAP, x_end_grid, y_end_grid] = MAP_update(pose, LidarData, odd_occ, odd_free, MAP)
    %MAP_UPDATE1 Summary of this function goes here
    %   Detailed explanation goes here
    log_odd_max = 77; 
    log_odd_min = -50;
    % 
    scan_angle = -135:0.25:135;
    scan_angle = scan_angle*pi/180;
    
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
    px_grid = ceil((px - MAP.xmin + 0.5*MAP.res) ./ MAP.res) ;
    py_grid = ceil((py - MAP.ymin + 0.5*MAP.res) ./ MAP.res) ;
    x_end_grid = ceil((x_end - MAP.xmin + 0.5* MAP.res) ./ MAP.res) ;
    y_end_grid = ceil((y_end - MAP.ymin + 0.5* MAP.res) ./MAP.res) ;
    %get empty cells
    [x_empty, y_empty] = getMapCellsFromRay(px_grid, py_grid, x_end_grid, y_end_grid);
    %update map likelihood
    IndEmpty = sub2ind([MAP.sizey, MAP.sizex], x_empty, y_empty);
    IndFull = sub2ind([MAP.sizey, MAP.sizex], x_end_grid, y_end_grid);
    %sharpening kernel
    
    
    subright = min(x_end_grid+1, MAP.sizex);
    subleft = max(x_end_grid-1, 1);
    subup = min(y_end_grid+1, MAP.sizey);
    subdown = max(y_end_grid-1, 1);
    
    Indright = sub2ind([MAP.sizex, MAP.sizey], subright, y_end_grid);
    Indleft = sub2ind([MAP.sizex, MAP.sizey], subleft, y_end_grid);
    Indup = sub2ind([MAP.sizex, MAP.sizey], x_end_grid, subup);
    Inddown = sub2ind([MAP.sizex, MAP.sizey], x_end_grid, subdown);
    
    
    Indru = sub2ind([MAP.sizex, MAP.sizey], subright, subup);
    Indlu = sub2ind([MAP.sizex, MAP.sizey], subleft, subup);
    Indrd = sub2ind([MAP.sizex, MAP.sizey], subright, subdown);
    Indld = sub2ind([MAP.sizex, MAP.sizey], subleft, subdown);
    
    EmptyXFull = intersect(IndEmpty, IndFull);
    IndEmpty = setdiff(IndEmpty, EmptyXFull);
    
    MAP.logmap(IndEmpty) = MAP.logmap(IndEmpty) + odd_free;
    MAP.logmap(IndFull) = MAP.logmap(IndFull) + 2.56*odd_occ;
    
    
    MAP.logmap(Indright) = MAP.logmap(Indright) - 0.18*odd_occ;
    MAP.logmap(Indleft) = MAP.logmap(Indleft) - 0.18*odd_occ;
    MAP.logmap(Indup) = MAP.logmap(Indup) - 0.18*odd_occ;
    MAP.logmap(Inddown) = MAP.logmap(Inddown) - 0.18*odd_occ;
    
    MAP.logmap(Indru) = MAP.logmap(Indru) - 0.1273*odd_occ;
    MAP.logmap(Indlu) = MAP.logmap(Indlu) - 0.1273*odd_occ;
    MAP.logmap(Indrd) = MAP.logmap(Indrd) - 0.1273*odd_occ;
    MAP.logmap(Indld) = MAP.logmap(Indld) - 0.1273*odd_occ;
    
    
    %saturatedd
    IndBig = MAP.logmap > log_odd_max;
    IndSmall = MAP.logmap < log_odd_min;
    MAP.logmap(IndBig) = log_odd_max;
    MAP.logmap(IndSmall) = log_odd_min;
    %update map
    MAP.map = int8(MAP.logmap > 50);
end

