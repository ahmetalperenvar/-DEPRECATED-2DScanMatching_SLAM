function [ C ] = remapMatrix( A, B )
%REMAPMATRIX given a matrix A and a map schema B, reorder the elements
%according to that schema
C = zeros(size(A,1),size(A,2));
for j = 1:size(A,1)
    C(j,:) = A(B(j),:);
end

end

