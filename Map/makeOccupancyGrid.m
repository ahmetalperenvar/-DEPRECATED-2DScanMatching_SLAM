function [ O ] = makeOccupancyGrid( A, PARAM )
%OCCUPANCYGRID Initialize an occupancy grid map using a (might be empty) clouds
%of points A and a resolution res. The size is enough to cover all the
%points plus a margin of size 1. The grid expans from xmin to xmax and ymin
%ymax. Values can be negatives and are relatives to robot position. the
%map is represented as a grid of xwide cells from 0 to xwide and y
%calculated as the same. Then, the respective cell where a point P falls is
%calculated taking into account the area represented by the map stored into
%the parameters PARAM. The robot is believed to be in the middle cell.
%Usually xmin and ymin are negative and equal -xmax and -ymax repsectively

% Depending on the type of the map we create in different ways. For
% probabilistic mode, the scan A needs to be presented in Polar
% Coordinates.

L = PARAM.L;
xmin = PARAM.xmin;
xmax = PARAM.xmax;
ymin = PARAM.ymin;
ymax = PARAM.ymax;


xwide = abs(xmax - xmin);
ywide = abs(ymax - ymin);


NX = floor(xwide / L)+1;
NY = floor(ywide / L)+1;

O.grid = ones(NX,NY)*0;
O.PARAM = PARAM;

O = fillOccupancyGrid(A,O, [0 0 0]);

end




