function [ B ] = popCovariance( A )
%CHECKCOVARIANCE Given a population of chromosomes, generate a covariance
% matrix useful to analyze convergence

meana = mean(A);

B = zeros(3,3);
for j= 1: size(A,1)
    t = A(j,:) - meana;
    t = t'*t;
    B = B+t;
end
B = B/size(A,1);

end


