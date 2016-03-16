function [ B ] = gridShift( A, x, y )
%GRIDSHIFT Shift a matrix by x and y cells respectively in x and y
%direction. The rest is set to zero. The cells of A falling outside the
%size are lost.

sizex = size(A,1);
sizey = size(A,2);

B = zeros(sizex,sizey);

for i=1:sizex
    ii=i+x;
    if ii>sizex || ii<=0
        continue
    end
    for j=1:sizey
        jj = j + y;
        if jj>sizey || jj<=0
            continue
        end
        
        B(ii,jj) = A(i,j);
    end
end

end

