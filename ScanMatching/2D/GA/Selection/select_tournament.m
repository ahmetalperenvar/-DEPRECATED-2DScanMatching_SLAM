function [ B ] = select_tournament( A, fit )
%SELECT_TOURNAMENT Tournament selection for Genetic Algorithms. Given a set A of
% points (chromosomes) and a fitting array representing the fitness value
% of each chromosome within our problem. Draw N new points using the
% Roulette Wheel system. The points are selected in couples.

% A good reference
% http://www.edc.ncl.ac.uk/highlight/rhjanuary2007g02.php/

nchr = size(A,1);

% We select only the best indexes according to a pressure (threshold) value
% to improve the goodness of the breeding population

press = mean(fit)*0.8
ksize = 10; % Tournament size
    
    B = zeros(nchr,3);
% We now want to generate a new set of surviving chromosomes with
% replacement (ie. a chromosomes can be chosen multiple times)
for j = 1:nchr
    
    mpool = [];
    
    
    % Choose KSIZE random elements to create a tournament
    for k = 1:ksize
        mpool(k) = fit(floor(rand*nchr)+1);
    end
    
    % Normalize the fitness value using the new surviving population
    sfit = sum(mpool);
    nfit = mpool./sfit;
    [nfit infit] = sort(nfit,'descend');

    nmp = size(mpool,1);
    
    % Select the best candidate using a probability to give chance to worse
    % candidates
    for k = 1:nmp
        v = rand(1) % Using a uniformely generated number, draw one value
        p = nfit(k)
        if v > p*((1-p)^(k-1)) % a candidate has probability to be chosen as p*((1-p)^j)
            break;
        end
        
    end
    B(j,:) = A(infit(k),:);
    
end

end


