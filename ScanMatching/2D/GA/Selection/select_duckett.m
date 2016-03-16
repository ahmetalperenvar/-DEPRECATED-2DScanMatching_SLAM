function [ B ] = select_duckett( A, fit )
%SELECT_DUCKETT selection process for genetic algorithms. Given a set A and
% the fitness values, a new population of same size is sorted according to
% the method used by

% Genetic Algorithm for Simultaneous Localization and Mapping
% Tom Duckett
% Centre for Applied Autonomous Sensor Systems
% Dept. of Technology, Orebro University
% SE-70182 Orebro, Sweden

% Values are normalized and resampled in the interval [ 0.5  1.5]

nchr = size(A,1);
sfit = sum(fit);
fit = fit./sfit;
[fit ifit] = sort(fit,'descend');
minf = fit(size(fit,1));
maxf = fit(1);

intv = (maxf - minf);

fit = 0.5 + ( (fit-minf) / intv);

% entries with integer part greater than 0 are automatically selected.
% A is remapped with the sorting information
A = remapMatrix(A,ifit);
B = zeros(nchr,3);
nchos = 0;
for j = 1:size(A,1)
    if (fit(j) >= 1)
        nchos = nchos+1;
        B(nchos,:) = A(j,:);
    end
end

remtc = size(A,1) - nchos;

% the remaining entries are drawn using the fractional part as probability
% to be selected
for k = 1:remtc-1
    for j = 1: (nchr-k+1)
        v = rand;
        if v < fit(j)
            B(nchos+k+1,:) = A(j,:);
            A(j,:) = [];
            break
        end
    end
end

end

