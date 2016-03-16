%Multiply to every row

function M = addRow(A,v)

    for i = 1:size(A,1)
       M(i,:) = v+A(i,:);
    end
end
