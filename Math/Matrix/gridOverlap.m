function [ score ] = gridOverlap( A, B )
%GRIDOVERLAP returns a score based on the overlap of two grids. The grids A
%and B must be the same size

gsx = size(B,1);
gsy = size(B,2);
score =0;
for i=1:gsx
    for j=1:gsy
        if A(i,j) > 0 && B(i,j) > 0
            score = score+1;
        end
    end
end

end

