function [R,t]=regist_besl(P1,P2)
%Given two sets of points, it computes the least square minimization to
%find the best rotation R and translation t that would match the two
%shapes.

% The article follow the algorithm described in: 
% Besl, P. J. and McKay, N. D.: A method for registration of 3-D shapes, IEEE Transactions on
% Pattern Analysis and Machine Intelligence 14(2) (1992), 239–256.

n=min(size(P1,2),size(P2,2));

% Find the mean of both point set, which will be their center of mass.
mp1(1)=mean(P1(1,:));
mp1(2)=mean(P1(2,:));
mp1(3)=mean(P1(3,:));

mp2(1)=mean(P2(1,:));
mp2(2)=mean(P2(2,:));
mp2(3)=mean(P2(3,:));

% Find the cross-covariance matrix of the two set with respect to their
% mean
Cp1p2=zeros(3,3);
for i=1:n
    Cp1p2=(P1(1:3,i)'-mp1)'*(P2(1:3,i)-mp2')'+Cp1p2;
end
Cp1p2=Cp1p2./n;

% Find the anti-symmetric matrix
A=Cp1p2-Cp1p2';

% .. and the vector delta 
D=[A(2,3);A(3,1);A(1,2)];

%.. to generate the matrix Q
Q=[trace(Cp1p2) D';D (Cp1p2+Cp1p2'-trace(Cp1p2)*eye(3))];

% we find the maximum eigenvalue which gives us the optimal rotation R
[eigVe,eigVa]=eig(Q);
eig_va=diag(eigVa);
[val,ind]=max(eig_va);
q=eigVe(:,ind);
R=q2R(q);

% We find the translation t rotating the center of mass of P1 and
% calculating the shift with P2
t=mp2'-R*mp1';
