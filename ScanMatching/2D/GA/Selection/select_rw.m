function [ B ] = select_rw( A, fit )
% SELECT_RW Roulette Wheel selection for Genetic Algorithms. Given a set A of
% points (chromosomes) and a fitting array representing the fitness value
% of each chromosome within our problem. Draw N new points using the
% Roulette Wheel system. The points are selected in couples.


% Normalize the fit  function. Each entry contains the sum of the previous
% fits normalized.

nchr = size(A,1);

sfit = sum(fit);sumfit = 0;
wfit = zeros(nchr,1);

for j = 1:nchr
    wfit(j) = sumfit + fit(j)/sfit;
    sumfit = wfit(j);
end

c1 = -1;

B = zeros(nchr,3);

for j = 1:nchr
    v = rand(1); % Using a uniformely generated number, draw one point
    if v < wfit(1)
        c1=1;
    end
    for h = 1:nchr-1
        % let's check in which interval it falls
        if v > wfit(h) && v < wfit(h+1)
            c1 = h+1;
        end
    end
    B(j,:) = A(c1,:);
end



end

