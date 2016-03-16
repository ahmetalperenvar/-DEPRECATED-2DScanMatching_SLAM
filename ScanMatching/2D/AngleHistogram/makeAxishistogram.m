function [ A ] = makeAxishistogram( Scan, a, bounds )
%MAKEANGLEHISTOGRAM Generates the X or Y histogram of the distribution of the
% points in the X and Y space using the resolution Opt.map.resolution

global Opt
cmres = Opt.map.resolution;


switch a
    
    case 'x'
        
        maxs = bounds(1,1);
        mins = bounds(1,2);
        A = zeros(1,ceil((maxs-mins)/cmres) + 1 ) ;
        for k = 1:size(Scan,1)
            j = min(ceil( (Scan(k,1) - mins) / cmres)+1, length(A));
            A(j) = A(j) + 1;
        end
        
    case 'y'
        maxs = bounds(2,1);
        mins = bounds(2,2);
        A = zeros(1,ceil((maxs-mins)/cmres) + 1 ) ;
        
        for k = 1:size(Scan,1)
            j = min(ceil( (Scan(k,2) - mins) / cmres)+1,length(A));
            A(j) = A(j) + 1;
        end
end

end

