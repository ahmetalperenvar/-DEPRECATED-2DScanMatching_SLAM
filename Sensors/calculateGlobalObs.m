function [ globalCart2D globalPolar2D ] = calculateGlobalObs( Raw, Rob )
%CALCULATEGLOBALOBS Calculates the global coordinates of a scan

    Rob.state.x = frameRef(Rob.state.x,Rob.state0);

    x = Rob.state.x(1) ;
    y = Rob.state.x(2) ;
    rT = r2Rt(Rob.state.x(3), [x y]);
   
    globalCart2D = zeros(size(Raw.localCart,1),2);
    globalPolar2D = zeros(size(Raw.localCart,1),2);
    
    for i=1:size(Raw.localCart,1)

         p_r = ( rT*[ Raw.localCart(i,1:2) 1]' )';
         globalCart2D(i,1:2) = p_r(1:2);
        [a r] =  cart2pol(globalCart2D(i,1), globalCart2D(i,2) );        
        globalPolar2D(i,1:2) = [a r];
    end
end

