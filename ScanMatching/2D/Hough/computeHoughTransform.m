% Computes the Hough Transform
function buffer = computeHoughTransform(points, thetaCells, rhoCells, rhoScale)

% Initialize buffer
buffer = zeros(thetaCells, rhoCells);

% For each input point
for i=1:size(points,2)
	point = points(:,i);
	
	% For each theta cell
	for thetaCell=1:thetaCells
		
		% Corresponding angle
		theta = thetaCell * pi * 2 / thetaCells;
		
		% Find rho = p.x * cos(theta) + p.y*sin(theta)
		rho = [cos(theta) sin(theta)] * point;
		
		% Transform into cells
		rhoCell = rho * rhoCells / rhoScale;
		
		% Put the cells with rho=0 at the middle of the buffer
		rhoCell = rhoCell + rhoCells/2;
		
		% We need an integer
		rhoCell = round(rhoCell);
		
		% If (thetaCell, rhoCell) is inside the buffer..
		if (rhoCell>=1) && (rhoCell<=rhoCells)
			
			% increase the cell value
			buffer(thetaCell, rhoCell) = buffer(thetaCell, rhoCell) + 1;
		end
	end
end
