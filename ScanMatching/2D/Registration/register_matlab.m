function [R t] = register_matlab(Ai, Bi, motion, Opt)
% REGISTER_MATLAB Uses Matlab Curve Fitting Toolbox to solve the
% minimization. It requires to have already an association set.
% P1(n) is associated with P2(n).
% Ai being the reference scan and Bi bein the new scan

%Select which points to use, in this case local cartesian point in
Al.cart = Ai.localCart;
Al.polar = Ai.localPolar;

Bl.cart = Bi.localCart;
Bl.polar = Bi.localPolar;

R = []; %Final rotation correction
P = Bl.cart; %Initial points to be aligned
itMax = Opt.scanmatcher.iterations;

Br = Opt.scanmatcher.Br;   % Angular tolerance
Br0 = Br;
alfaBr = 1/(2.5*itMax);
Br = Br0*exp(1)^alfaBr;
Br = 0;
it=1;       %iteration control

sizeB = size(Bl.cart,1);

%Matrix initialization
B.cart = zeros(sizeB, 2);
B.polar = zeros(sizeB, 2);
C = zeros(sizeB, 3);
err = zeros(sizeB, 1);



%Motion extraction from robot

u = motion.con.u;
pose = motion.frame.t;
lrangle = u(2);
x0 =  cos(lrangle)*u(1);
y0 =  sin(lrangle)*u(1);
yaw0 = u(3);

x=x0;
y=y0;
yaw=yaw0;

lasterror = [];
B.cart(:,1:2) = Bl.cart(:,1:2);B.polar(:,1:2) = Bl.polar(:,1:2);
xtot = 0;
ytot = 0;
rtot = 0;
% rTn = [cos(yaw) -sin(yaw) x; sin(yaw) cos(yaw) y; 0 0 1];


% B referenced to A coordinate system
% for i = 1:sizeB
%     D.cart(i,1:3) = (rTn*[B.cart(i,1:2) 1]')' ;
%     th = normAngle(B.polar(i,1) + yaw);
%     ro = B.polar(i,2);
%     rc = cos(th)*ro + x;
%     rs = sin(th)*ro + y;
% 
%     D.polar(i,2) = sqrt(rc^2 + rs^2);
%     D.polar(i,1) = atan2(rs,rc);
% end
% 
% % Compute the association set
% [merr assp] = cpAssociation(D,Al, Br);
% assp(:,3) =  pose(3);



    function [v] = fmin(X)
        rTn = [cos(X(3)) -sin(X(3)) X(1); sin(X(3)) cos(X(3)) X(2); 0 0 1];
        
        % B referenced to A coordinate system
        for i = 1:sizeB
            D.cart(i,1:3) = (rTn*[B.cart(i,1:2) 1]')' ;
            th = normAngle(B.polar(i,1) + yaw);
            ro = B.polar(i,2);
            rc = cos(th)*ro + x;
            rs = sin(th)*ro + y;

            D.polar(i,2) = sqrt(rc^2 + rs^2);
            D.polar(i,1) = atan2(rs,rc);
        end

        % Compute the association set
        [merr assp] = cpAssociation(D,Al, Br);
        assp(:,3) =  pose(3);
        
        v=[];
        % B referenced to A coordinate system
        for k = 1:size(assp,1)
%             F(k,1:3) = (rTn*[B.cart(k,1:2) 1]')' ;
            v = [v; assp(k,1:2)-D.cart(k,1:2) ];
        end

    end


[T res] = lsqnonlin(@fmin, [x0 y0 yaw0]);
T = T - [x0 y0 yaw0];
R = e2q([0 0 T(3)]);
t = [T(1:2) 0];
end