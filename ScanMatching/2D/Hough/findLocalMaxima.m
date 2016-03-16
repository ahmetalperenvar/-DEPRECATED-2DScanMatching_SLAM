% Find the local maxima of a sequence; it returns
% the indexes in descending order
% 
% nm     maximum number of maxima returned
%
function maxima = findLocalMaxima(sequence, nm)
	
	% This is a bit of black magic; these are the beauties of Matlab
	indexes = find(diff(sign(diff(sequence,1)))==-2);
	
	values = sequence(indexes);
	
	[unused, sortedIndexes] = sort(values,'descend');
	
	maxima = indexes(sortedIndexes);
	
	if size(maxima,2)>nm
		maxima = maxima(1:nm);
	end
