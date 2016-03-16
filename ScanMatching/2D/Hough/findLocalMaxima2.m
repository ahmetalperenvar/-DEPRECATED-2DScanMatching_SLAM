% Find the local maxima of a sequence; it returns
% the indexes in descending order
% 
% nm     maximum number of maxima returned
%
function maxima = findLocalMaxima2(matrix, nm)
	
	[i,j] = find(imregionalmax(matrix));
	
	for a=1:size(i,1)
		values(a)=matrix(i(a),j(a));
	end
	
	
	[unused, sortedIndexes] = sort(values,'descend');
	
	maxima = [ i(sortedIndexes)'; j(sortedIndexes)'];
	
	if size(maxima,2)>nm
		maxima = maxima(:,1:nm);
	end
	
