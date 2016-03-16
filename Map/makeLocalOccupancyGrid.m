function [  SubGrids ] = makeLocalOccupancyGrid( Scan, PARAM )
%MAKELOCALOCCUPANCYGRID returns an occupancy grid out of a range finder
%scan. The grid size is dynamic and depends on the cartesian area covered
%by the points in Scan. We suppose the points to be ordered by angle.
%Adjacent points are neighbours. The grid has a probabilistic fashion, each
%cell is filled with its occupancy probability modeled by a gaussian
%derivied by the number of n points which fall inside it. 

% SUBGRIDS is a set of 4 occupancy grid maps defined using a small shift
% over the original center of the grid. 

L = PARAM.L;
xmin = PARAM.xmin;
xmax = PARAM.xmax;
ymin = PARAM.ymin;
ymax = PARAM.ymax;

sscan = size(Scan,1);


xwide = (xmax - xmin);
ywide = (ymax - ymin);


NX = ceil(xwide / L);
NY = ceil(ywide / L);

% we create the occupancy grid structures, each cell is defined by a mean
% and covariance pdf
ogrid.mean = 0;
ogrid.cov = zeros(2,2);

ogrid1.grid(1:NX, 1:NY) = ogrid;
ogrid2.grid(1:NX, 1:NY) = ogrid;
ogrid3.grid(1:NX, 1:NY) = ogrid;
ogrid4.grid(1:NX, 1:NY) = ogrid;
ogrid_tot.grid(1:NX, 1:NY) = ogrid;

%Initialize the cells to group the points togheter
pgrid0.points = [];

pgrid1.grid(1:NX, 1:NY) = pgrid0;
pgrid2.grid(1:NX, 1:NY) = pgrid0;
pgrid3.grid(1:NX, 1:NY) = pgrid0;
pgrid4.grid(1:NX, 1:NY) = pgrid0;

%Group scans according to the cell they fall in, Calculates 4 grids. The
%second grid is shifted horizontally of L/2, the third is shifted
%vertically of the same amount, and the last one is shifted both vertically
%and horizontally of L/2 as well.

for i = 1:sscan
    x = Scan(i,1);
    y = Scan(i,2);
    
    
    %Calculate the cell where the point (x,y) should fall in.
    
    ix1 = min(floor( (-(xmin) + x) / L)+1,NX);
    iy1 = min(floor( (-(ymin) + y) / L)+1,NY);
    
    ix2 = min(floor( (-(xmin) + x + L/2) / L)+1,NX);
    iy2 = iy1;
    
    ix3 = ix1;
    iy3 = min(floor( (-(ymin) + y + L/2) / L)+1,NY);
    
    ix4 = ix2;
    iy4 = iy3;
    
    % and add it to the respective map
    rest = pgrid1.grid(ix1,iy1).points;
    pgrid1.grid(ix1,iy1).points = [rest; x y];
    
    rest = pgrid2.grid(ix2,iy2).points;
    pgrid2.grid(ix2,iy2).points = [rest; x y];
    
    rest = pgrid3.grid(ix3,iy3).points;
    pgrid3.grid(ix3,iy3).points = [rest; x y];
    
    rest = pgrid4.grid(ix4,iy4).points;
    pgrid4.grid(ix4,iy4).points = [rest; x y];
end

grids = [pgrid1 pgrid2 pgrid3 pgrid4];
ogrids = [ogrid1 ogrid2 ogrid3 ogrid4];


% for each cell, calculate its mean and covariance using its containing
% points. If the number of points is smaller than 3 the cell results
% unoccupied. 
hh = 0;
tots = 0;
for i = 1:NX
    
    for j = 1:NY

        %For every cell calculate the four occupancy maps
        for kk = 1:size(grids,2)
            pgrid = grids(kk).grid;

            np = size(pgrid(i,j).points,1);
            mpoints = 0;
            covpoints = zeros(2,2);
            tots = tots +np;
            
            if np >= 3
             
                hh = hh+1;
                %Calculate mean and covariance of all the points in the cell
                mpoints = mean(pgrid(i,j).points);
                covpoints = cov(pgrid(i,j).points);
                %covpoints = zeros(2,2);
%                 for k = 1:np
%                     rest = pgrid(i,j).points(k,:) - mpoints;
%                     rest = rest'*rest;
%                     covpoints = covpoints + rest;
%                 end
%                 covpoints = covpoints/(np-1);
                
%                 mpoints
%                 covpoints
%                 pgrid(i,j).points
%                 j
%                 i
            end
            
            %update the points in each map...
            mpoints;
            covpoints;
            ogrids(kk).grid(i,j).mean = mpoints;
            ogrids(kk).grid(i,j).cov = covpoints;
        end
        
        %and to the total map which is the sum of the previous four
%         ogrid_tot.grid(i,j).mean = sum(ogrids(1:4).mean);
%         ogrid_tot.grid(i,j).cov = sum(ogrids(1:4).cov);
    end


end

%displayMultOccGrid(ogrids);
SubGrids = [ogrids(1) ogrids(2) ogrids(3) ogrids(4)];
end
