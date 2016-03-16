% HANNING.M
%
% COPYRIGHT : (c) NUHAG, Dept.Math., University of Vienna, AUSTRIA
%             http://nuhag.eu/
%             Permission is granted to modify and re-distribute this
%             code in any manner as long as this notice is preserved.
%             All standard disclaimers apply.
%
% HANNING.M     - returns the N-point Hanning window in a column vector.
%
% Input		: n = signal length (or column vector?) 
%
% Output	: N-point Hanning window
%
% Usage		: w = hanning(n);  or 2D:  hanning(n1,n2);  
%
% Comments	: empty argument shows plot, modification of original MATLAB M-file      

% HGFei 

function w = hann2d(n,n2)

if nargin == 2;  
    w1 = hann2d(n);
    w2 = hann2d(n2);
    w = w1(:) * w2;
    return;
end; 
    
if nargin == 0
 help hann2d;
 return;
end;

[h,w] = size(n);
if h*w > 1;
n = max(h,w);
end; 
 
u = .5*(1 - cos(2*pi*(0:n-1)'/(n-1)));

if h*w > 1;
w = zeros(h,w);
w(:) = u; 
else
w = u(:).';    
end;  
