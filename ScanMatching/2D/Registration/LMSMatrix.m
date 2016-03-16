function [out,estimation] = LMSMatrix(assoc)
global PARAM
PARAM.L = 3;
L2 = PARAM.L^2;

new = [assoc.new'];
ref = [assoc.ref'];

r2 = ref.^2;
n2 = new.^2;

rxry = ref(1,:).*ref(2,:);
nxrx = new(1,:).*ref(1,:);
nxry = new(1,:).*ref(2,:);
nyrx = new(2,:).*ref(1,:);
nyry = new(2,:).*ref(2,:);


k = r2(1,:)+r2(2,:)+L2;

ds = nyry+nxrx;
dsd = ds./k;
r2dsd = ref.*[dsd;dsd];

bs = nxry-nyrx;
bsd = bs./k;

A = zeros(3,3); b = zeros(1,3);
for i = 1:size(assoc.ref,1)
    A(1,1) = A(1,1) + (1-r2(2,i)/k(i));
    A(1,2) = A(1,2) + rxry(i)/k(i);
    A(1,3) = A(1,3) + (-new(2,i) + ref(2,i)*dsd(i) );
    
    A(2,2) = A(2,2) + (1-r2(1,i)/k(i));
    
    A(2,3) = A(2,3) + (new(1,i) - ref(1,i)*dsd(i) );
    
    A(3,3) = A(3,3) + (n2(1,i)+n2(2,i)-(ds(i)^2)/k(i));
    
    b(1) = b(1) + ( new(1,i)-ref(1,i)-(ref(2,i)*bsd(i)) );
    b(2) = b(2) + ( new(2,i)-ref(2,i)+(ref(1,i)*bsd(i)) );
    b(3) = b(3) + (bs(i)*(-1+dsd(i)));
end

A(2,1) = A(1,2);
A(3,1) = A(1,3);
A(3,2) = A(2,3);


% For some reasons if the vector b is too low the estimation converge to a
% wrong minimum, probably cuz of the singularity.
%if sum( abs(b) ) > 0.01
    estimation = -A\b';
%else
%    estimation =  [0 0 0]
%end

out = 1;
