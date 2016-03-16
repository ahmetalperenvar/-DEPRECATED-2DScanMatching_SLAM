function [ B ] = select_truncate( A, fit )
%SELECT_TRUNCATE Simply use a threshold to eliminate the values with worse
%entry. The value is calculated using the mean of the population and it is
%scaled by a factor SC, the smaller it gets the more tollerant the
%selection will be. Using strict tolerance increase fast convergence

sc = 0.8;
sfit = sum(fit);
m = mean(fit./sfit) * sc;
B = [];
nchos = 0;
for j = 1:size(A,1)
    if (fit >= m)
        nchos = nchos+1;        
        B(nchos,:) = A(j,:);
    end
end

end

