function [R t] = plRegistration(P1,P2)
%PLASSOCIATION Summary of this function goes here
%   Detailed explanation goes here

	g = zeros(4,1);
	
	for k=1:size(P1,1)
		M_k = [eye(2) P1(k,1) -P1(k,2); P1(k,2) P1(k,1)];
        n = -(P2(k,1)-P2(k,3) ) / (P2(k,2)-P2(k,4) );
        n = [ cos(n) sin(n) ];
        nn = n*n';
		M = M + M_k'* nn *M_k;
		g = g + (- 2 * P2(k,1:2)' * nn * M_k)';
	end
	
	W = [ zeros(2,2) zeros(2,2); zeros(2,2) eye(2)];
	
	%% This is the function that we want to minimize 
	h = @(l) g' * inv(M+2*l*W) * W * inv(M+2*l*W)' * g - 1;
	
	%% Partition M in 4 submatrixes:
	
	M = 2*M;
	% M = [A B; C D]
	A = M(1:2,1:2);
	B = M(1:2,3:4);
	D = M(3:4,3:4);
	inA = inv(A);
    
	S = D - B' * inA * B;
	Sa = inv(S) / det(S);
		
	g1 = g(1:2); g2=g(3:4);

   p7 = [( g1'*(inA*B*  4    *B'*inA)*g1 + 2*g1'*(-inA*B*  4   )*g2  + g2'*( 4   )*g2) ...
	      ( g1'*(inA*B*  4*Sa *B'*inA)*g1 + 2*g1'*(-inA*B*  4*Sa)*g2  + g2'*( 4*Sa)*g2) ...
			( g1'*(inA*B* Sa*Sa *B'*inA)*g1 + 2*g1'*(-inA*B* Sa*Sa)*g2  + g2'*(Sa*Sa)*g2)];
			
	p_lambda = [4 (2*S(1,1)+2*S(2,2)) (S(1,1)*S(2,2)-S(2,1)*S(1,2))];
	Ptot = polyadd(p7, -conv(p_lambda,p_lambda)) ;

	% Find largest real root of Ptot
	r = roots(Ptot);
	lambda = 0; found = 0;
	for i=1:4
		if isreal(r(i)) && (not(found) || r(i)>0)
			lambda = max(lambda, r(i));
			found = 1;
		end
	end
	
	x = -inv(M + 2 * lambda * W) * g;
	theta = atan2(x(4),x(3));
	
	R = e2q([0 0 theta]);
	t = [x(1) x(2) 0];



