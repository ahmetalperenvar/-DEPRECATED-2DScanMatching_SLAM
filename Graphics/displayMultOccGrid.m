function [ out ] = displayMultOccGrid( grids )
%DISPLAYOCCGRID display the sum of occupancy grids

sx = size(grids(1).grid,1);
sy = size(grids(1).grid,2);
vals = zeros(sx,sy);

for i = 1: sx
    for j = 1: sy
        for k = 1:size(grids,2)
            vals(i,j) = sum(sum(grids(k).grid(i,j).mean));
            if vals(i,j) == 0
                vals(i,j) = -1;
            else
                vals(i,j) = 1;
            end
        end
    end
end

surf(1:sx,1:sy,vals');
shading interp
axis equal
out=1;
end

