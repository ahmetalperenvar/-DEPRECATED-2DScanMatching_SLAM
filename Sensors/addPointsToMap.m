

function [ Map filtered ] = addPointsToMap(Points,Map,Tim,Rob)

%Add points to a map depending on its type. Only list of points. 

        filtered = Rob.filtered;
        filtered.localPolar=[];
        filtered.localCart=[];
switch(Map.type)
    
    %For occupancy grid 2D. TO DO. Just point cloud so far
    case 'occgrid'
        
        actualdata = size(Points.localCart,1);
        
        if(actualdata <= 0)
            return
        end
        
        [data polarp] = calculateGlobalObs(Points,Rob);
        covm = Points.covm;
        
        Map.grid = [Map.grid; data]; %( ( (Tim.currentFrame-1) * datasize + 1) : (Tim.currentFrame*datasize)  ,:)  = data;       
        Map.prev.gridlocal(Tim.currentFrame).points = Points.localCart;
        Map.prev.gridlocalpolar(Tim.currentFrame).points =  Points.localPolar;
        Map.prev.gridpolar(Tim.currentFrame).points = polarp;
        Map.prev.gridcovm(Tim.currentFrame).points = covm;
        Map.prev.written(Tim.currentFrame) = actualdata;
        

             
        
    otherwise
        
          error('ERROR: Unknown Map Type ''%s''.',Rob.motion);

end
