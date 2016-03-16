%Add to every row

function M = multRow(A,v)

    for i = 1:size(A,1)
       M(i,:) = A(i,:)*v;
    end
end
