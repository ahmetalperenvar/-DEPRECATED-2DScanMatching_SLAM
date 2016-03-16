function [ B ] = pprocess( A, thr, ncolors )
%PPROCESS Filter the image A using a threshold THR and resampling in
%NCOLORS



mx =  max(max(A));

dif = ncolors/(mx - thr);

for k = 1:size(A,1)
    for v = 1:size(A,2)
        if A(k,v) > thr
            B(k,v) = floor((A(k,v) - thr) * (dif));
        else
            B(k,v) = 0;
        end
    end
end


end

