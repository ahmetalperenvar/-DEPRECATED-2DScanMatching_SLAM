function [ j ] = searchPair( A, P )
%SEARCHPAIR Search a pair P of numbers in the array A. They must be
%consecutive. Returns I as the position within the array

for j = 1:size(A,1)
    if (P(1) == A(j,1) || P(2) == A(j,2)) &&...
        (P(2) == A(j,1) || P(1) == A(j,2))
        return
    end
end
j = -1;
end

