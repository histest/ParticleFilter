function [x_grid, y_grid] = pos2grid( x, y, MAP )
    %POS2GRID converts coordinates to grids on the map here
    
    if x > MAP.xmax || y > MAP.ymax || x < MAP.xmin || y < MAP.ymin
        x_grid = -1;
        y_grid = -1;
    else
        x_grid = ceil((x - MAP.xmin) ./ MAP.res + 1);
        y_grid = ceil((y - MAP.ymin) ./ MAP.res + 1);
    end
        
    
end

