function [ out ] = displayOccGrid( grids )
%DISPLAYOCCGRID display the occupancy grid

sx = size(grids.grid,1);
sy = size(grids.grid,2);
vals = zeros(sx,sy);

for i = 1: sx
    for j = 1: sy
            vals(i,j) = grids.grid(i,j);
    end
end

surf(1:sx,1:sy,vals');
imshow(vals,[-1 1]);
g = colormap('gray');
g(1,:) = [ 1 0 0];
%g(2:127,:) = [ 0.2 0.2 0.7];
colormap(g);
% shading interp
% axis equal
out=1;
end

