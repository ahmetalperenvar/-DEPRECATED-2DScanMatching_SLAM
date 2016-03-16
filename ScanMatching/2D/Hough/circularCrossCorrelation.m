% Computes the circular cross-correlation between two sequences
%
% a,b             the two sequences
% normalize       if true, normalize in [0,1]
%
function c = circularCrossCorrelation(a,b,normalize)

for k=1:length(a)
    c(k)=a*b';
    b=[b(end),b(1:end-1)]; % circular shift
end

if normalize
	minimum = min(c);
	maximum = max(c);
	c = (c - minimum) / (maximum-minimum);
end
