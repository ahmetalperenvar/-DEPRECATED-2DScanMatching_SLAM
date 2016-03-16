function [out,solution,errorK1,nConverged] = pICRegistration(assoc,motion,errorK1,nConverged,tmp)

errRatio                    = 0.0001;
error                         = [0.0001 0.0001 0.0001];
nIterationSmoothConvergence   = 2;
%motion_estimation = qpose3D2epose2D(motion.x);

assoc.new = assoc.new';
assoc.ref = assoc.ref';

if (size(assoc.new,2) == 0)
    error('No associations avaiable.');
end


E = zeros(2*size(assoc.new,2),1);
H = zeros(2*size(assoc.new,2),3);
C = zeros(2*size(assoc.new,2));

for i = 1:size(assoc.new,2)
    E(2*i-1:2*i) = -(assoc.ref(:,i)-assoc.new(:,i))+assoc.Jq(:,:,i)*motion.estimation';
    H(2*i-1:2*i,:) = assoc.Jq(:,:,i);
    C(2*i-1:2*i,2*i-1:2*i) = assoc.Pe(:,:,i);
end

invC = C\eye(size(C));
qmin = (H'*invC*H)\H'*invC*E;

% Compute error
e = Composition(qmin,assoc.oldn);
e = (e - assoc.ref).^2;
err = sum(sum(e,2));

% Convergence criteria
errorRatio = err/errorK1;
diff = abs(motion.estimation - qmin');
if (abs(1-errorRatio) <= errorRatio) || (diff(1) < error(1) && diff(2) < error(2) && diff(3) < error(3))
    nConverged = nConverged+1;
else
    nConverged = 0;
end

% Build solution
errorK1 = err;
solution = qmin;

% Smooth convergence criterion
if (nConverged > nIterationSmoothConvergence)
    out = 1;
else
    out = 0;
end


