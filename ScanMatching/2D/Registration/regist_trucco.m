function [R,res]=regist_trucco(P2,P1)
% P1 3xn 
% P2 3xn 

n=size(P1,2);
for i=1:n
	A(i,:)=[P1(:,i)' 0 0 0 0 0 0];
      A(n+i,:)=[0 0 0 P1(:,i)' 0 0 0];
      A(2*n+i,:)=[0 0 0 0 0 0 P1(:,i)'];
      b(i,1)=P2(1,i);
      b(n+i,1)=P2(2,i);      
      b(2*n+i,1)=P2(3,i);      
end

if rcond(A'*A)>0.00001,
    r = (inv(A'*A))*A'*b;
else
    AA=[A -b];
    [V,D]=eig(AA'*AA);
    [nm,im]=min(sum(D));
    r=V(:,im)./V(10,im);
    r=r(1:9);
end


R=[r(1:3)';r(4:6)';r(7:9)'];
% size(A)
% size(r)
% size(b)
res=A*r-b;

