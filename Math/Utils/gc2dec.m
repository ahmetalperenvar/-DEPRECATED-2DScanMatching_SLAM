function gcode = gc2dec(dec)

% Description: This function converts a decimal number to its equivalent
% gray code representation.
tmp = dec;   idv = 1;
while 1
   dv = bitshift(tmp,-idv);
   tmp = bitxor(tmp,dv);
   if dv <= 1 || dv == 16
       break;
   end
   idv = bitshift(idv,1);
end
gcode = tmp;