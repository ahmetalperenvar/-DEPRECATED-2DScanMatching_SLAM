function [ A B ] = filterAssMatrix( A, B, i )
%FILTERASSMATRIX Given two matrices of points A and B with B representing
%the association set of A. Removes all the indeces from A and B which are
%not associated. ie when an element of i = -1

ix = find(i == -1);
if ~isempty(A)
A(ix,:) = [];
end
if ~isempty(B)
B(ix,:) = [];
end

end

