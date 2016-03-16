function [ b ] = arrayshift( a , n )
%ARRAYSHIFT Summary of this function goes here
%   Detailed explanation goes here

for j = 1:size(a,2)
   b(j) = a( mod(j+n, size(a,2) )+1); 
end

end

