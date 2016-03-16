function [ dec ] = dec2gc(gcd)
%GC2DEC Returns the inverse of the Gray code representation of a decimal

dec = bitxor(gcd,bitshift(gcd,-1));

end

