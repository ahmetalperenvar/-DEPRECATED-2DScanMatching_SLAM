function [ ix iy ] = getCoordinatesSubgrid( x, y, i, PARAM )
%GETCOORDINATESSUBGRID Returns the correct coordinates according to the
%subgrid I. Every subgrid has an own shift
L = PARAM.L;
xmin = PARAM.xmin;
xmax = PARAM.xmax;
ymin = PARAM.ymin;
ymax = PARAM.ymax;

xwide = xmax - xmin;
ywide = ymax - ymin;

NX = floor(xwide / L);
NY = floor(ywide / L);

%Calculate the cell where the point (x,y) should fall in.
if(i == 1)
    ix = min(floor( (-(xmin) + x) / L)+1,NX);
    iy = min(floor( (-(ymin) + y) / L)+1,NY);
elseif(i==2)
    ix = min(floor( (-(xmin) + x + L/2) / L)+1,NX);
    iy = min(floor( (-(ymin) + y) / L)+1,NY);
elseif(i==3)
    ix = min(floor( (-(xmin) + x) / L)+1,NX);
    iy = min(floor( (-(ymin) + y + L/2) / L)+1,NY);
else
    ix = min(floor( (-(xmin) + x + L/2) / L)+1,NX);
    iy = min(floor( (-(ymin) + y + L/2) / L)+1,NY);
end

end

