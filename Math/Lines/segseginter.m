function [ out , pout, di] = segseginter( p1,p2,p3,p4 )
%SEGSEGINTER Summary of this function goes here
%   Detailed explanation goes here

%    Calculate the line segment PaPb that is the shortest route between
%    two lines P1P2 and P3P4. Calculate also the values of mua and mub where
%       Pa = P1 + mua (P2 - P1)
%       Pb = P3 + mub (P4 - P3)
%    Return FALSE if no solution exists.

%    XYZ p13,p43,p21;
%    double d1343,d4321,d1321,d4343,d2121;
%    double numer,denom;
% EPS = 1;
%    p13(1) = p1(1) - p3(1);
%    p13(2) = p1(2) - p3(2);
%    p43(1) = p4(1) - p3(1);
%    p43(2) = p4(2) - p3(2);
%    
%    
%    if (abs(p43(1)) < EPS && abs(p43(2)) < EPS)
%       out = (0);
%       return;
%    end
%    p21(1)= p2(1)- p1(1);
%    p21(2) = p2(2) - p1(2);
% 
%    if (abs(p21(1)) < EPS && abs(p21(2)) < EPS )
%       out = 0;
%       return
%    end
pout = [];
di = 0;
   d =   (p4(2) - p3(2)) * (p2(1) - p1(1)) - (p4(1) - p3(1)) * (p2(2) - p1(2));
   n_a = (p4(1) - p3(1)) * (p1(2) - p3(2))  - (p4(2) - p3(2)) * (p1(1) - p3(1));
   n_b = (p2(1) - p1(1)) * (p1(2) - p3(2))  - (p2(2) - p1(2)) * (p1(1) - p3(1));


   %denom = d2121 * d4343 - d4321 * d4321;
   if (abs(d) == 0)
      out = 0;
      return
   end
   
   ua = n_a / d;
   ub = n_b / d;

   
         if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1)
         
            pout(1) = p1(1) + (ua * (p2(1) - p1(1)));
            pout(2) = p1(2) + (ua * (p2(2) - p1(2)));
            di = sqrt( (pout(1) - p2(1))^2 + (pout(2) - p2(2))^2 );
            out = 1;
            return
         end
         out = 0;



end

