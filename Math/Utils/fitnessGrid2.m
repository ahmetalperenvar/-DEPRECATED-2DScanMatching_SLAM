function [ score A] = fitnessGrid2( A, B, motion )
%FITNESSGRID defines a fitness function for the grid B. It evaluates how
%good the scan A, applied the transformation MOTION, fits in the grid. It
%counts without any explicit association how many cells overlap. A cell is
% said to overlap with another if it falls under a range of a 5*5 area.
x0 = motion(1);
y0 =  motion(2);
yaw0 = motion(3);
A = applyTransform2Scan2D(A, yaw0, [x0 y0]);
sizen = size(A,1);
score = 0;
gr = B.grid;
gsx = size(B.grid,1);
gsy = size(B.grid,2);
sb = 1;


% Check every point in the scan A
for i = 1:sizen
    [a b] = getCoordinates(A(i,:),B.PARAM);
    % Check the surraounding area
    scored = 0;
    for j = 1:sb*2
        for k = 1:sb*2
            % Restrict the search area inside the grid map
            ix = a - sb + k ;
            iy = b - sb + j ;
            if ix > gsx || ix <= 0 ||  iy > gsy || iy <= 0
                continue
            end

            if gr(ix,iy) == 1
                if ~((-sb + k) == 0)
                    penalty = 0.2;
                else
                    penalty = 0;
                end

                if ~((-sb + j) == 0)
                    penalty = penalty+ 0.2;
                end                   
                score = score + 1 - penalty; % - (0.1*(-sb+k))*(1*(-sb+j))  ;
                scored = 1;
            end
        end
        if scored
            continue
        end
    end
    
   
end

end

