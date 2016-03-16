% Computes the Hough Spectrum from the buffer.
%
% shouldNormalize  if true, normalize result in [0,1]
function spectrum = computeHoughSpectrum(HTbuffer, shouldNormalize)

% Computing the spectrum is very easy: 
% we square each cell and then sum over rho
spectrum = sum(( HTbuffer .^ 2 )');

if shouldNormalize
	spectrum = normalize(spectrum);
end

