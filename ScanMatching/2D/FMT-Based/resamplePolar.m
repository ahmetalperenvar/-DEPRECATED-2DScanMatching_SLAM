function [ B ] = resamplePolar( A, N )
%RESAMPLEPOLAR Resample a magnitude spectra into a polar spectra of size N
gridsize = [N N];
imagesize = [size(A,1) size(A,2)];
B = zeros(N,N);

% Resample into polar coordinates
for k = 1:gridsize(1)
    for j = 1:gridsize(2)
        K = floor( (((imagesize(1)/2)*k)/gridsize(1))*cos((pi*j)/gridsize(2)) + imagesize(1)/2 ) + 1;
        J = floor( (((imagesize(1)/2)*k)/gridsize(1))*sin((pi*j)/gridsize(2)) + imagesize(1)/2 ) + 1;
        K = min(K,size(A,1));
        J = min(J,size(A,2));
        B(k,j) = A(K,J);
    end
end

end

