function [ c, in ] = nMaxima( A, n )
%NMAXIMA Returns the first n maxima of a array/matrix. If it is a matrix
% It opereates through all the matrixignoring its dimension. 

if nargin==1
    n = 1;
end

rows = size(A,1);
cols = size(A,2);

if size(A,2) > 1
    
    for i=1:size(A,2)
        B = [B A(1,:)];
    end
else
    B = A;
end

c = zeros(n);
in = zeros(n,2);

for i=1:n
    
    [C I] = max(B);
    B(I) = -inf;
    ii(1,2) = floor(I/rows);
    ii(1,1) = I - i(2) * cols; 
    
    c(i) = C;
    in(i) = ii;
end


end

