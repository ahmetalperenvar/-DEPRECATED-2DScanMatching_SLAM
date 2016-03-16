% Find the local maxima of a sequence; it returns
% the indexes in descending order
% 
% nm     maximum number of maxima returned
%
function maxima = findLocalMaxima(sequence, nm)
	
	extended=[sequence(end)  sequence sequence(1)];
	m=findLocalMaxima(extended, nm);
	maxima =  m-1;
	
