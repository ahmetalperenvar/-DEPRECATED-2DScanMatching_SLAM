function [out,solution,errorK1,nConverged] = registerMbICP(assoc,motion,errorK1,nConverged)

PARAM.Bw                            = 6.23;
PARAM.Br                            = 2.1;
PARAM.L                             = 3;
PARAM.step                          = 1;
PARAM.maxDistanceInterpolation      = 0.5;
PARAM.filter                        = 1;
PARAM.projectionFilter              = 0; %!! test with 0
PARAM.associationError              = 0.1;
PARAM.maxIterations                 = 250;
PARAM.errorRatio                    = 0.0001;
PARAM.error                         = [0.0001 0.0001 0.0001];
PARAM.nIterationSmoothConvergence   = 2;

PARAM.deviation.sensor              = [0.0262 0.05]; % std deviation [angular(rad) range(m)] [+/-1.5� +/-5cm]
PARAM.deviation.motion              = [0.1 0.1 deg2rad(10)]; % motion std deviation [x(m) y(m) yaw(rad)]

if (size(assoc,2) == 0)
    error('No associations avaiable.');
    out =  -1;
    return
end

cpAssoc = assoc;
[out,estimation] = LMSMatrix(cpAssoc);

if(out == -1)
    out = -1;
    solution =[];
    return;
end

% Compute associatons' error
cosy = cos(estimation(3)); siny = sin(estimation(3));
dtx = estimation(1); dty = estimation(2);

ref = [assoc.ref'];
new = [assoc.new'];

errorTmp = 0;
for i = 1:size(ref,2)
    tmp1 = new(:,i)-ref(:,i); %*cosy-new(2,i)*siny+dtx-ref(1,i);
    %tmp2 = new(1,i)*siny-new(2,i)*cosy+dty-ref(2,i);
    errorTmp = errorTmp+sqrt(tmp1(1)^2 + tmp1(2)^2);
end

% Build solution
errorK1 = errorTmp;

% Motion referenced to computed estimation
%eTm = [cos(estimation(3)) -sin(estimation(3)) estimation(1); sin(estimation(3)) cos(estimation(3)) estimation(2); 0 0 1];

eTm = [cos(estimation(3)) -sin(estimation(3)) 0; sin(estimation(3)) cos(estimation(3)) 0; 0 0 1];
solution = eTm*[estimation(1:2,1); 1];
%solution(3,1) = estimation(3)+motion(3);
%solution = solution + estimation;
solution(3,1) = Normalize(estimation(3));


%solution = motion + estimation;
solution=solution';
% Smooth convergence criterion
if (nConverged > PARAM.nIterationSmoothConvergence)
    out = 1;
else
    out = 0;
end
