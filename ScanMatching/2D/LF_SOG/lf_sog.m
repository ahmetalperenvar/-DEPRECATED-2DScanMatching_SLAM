% This algorithm works similarly to NDT but it does not work with grids.
% Likelihood ﬁelds are, in this case, deﬁned using the two
% scans. The minimizing function is, this time, deﬁned as
% the exponential of the distances between a point in Snew
% and a set of points in Sref . The set of points in Sref is
% decided using an euclidean distance threshold.

% P. Biber, S. Fleck, and W. Strasser, “A probabilistic framework for robust
% and accurate matching of point clouds,” Pattern Recognition, pp. 480–
% 487, 2004.

function [R t NI] = lf_sog(ScanRef, ScanNew, motion)
% In:
%           Ai: localCart[2xN], points cartesian
%           Bi: localCart[2xN], points cartesian
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations

global DEBUG Opt

itMax = Opt.scanmatcher.iterations;

% we only use cartesian points
ScanNew = downsample(ScanNew.localCart,3);
ScanRef = downsample(ScanRef.localCart,3);
sizenew = size(ScanNew,1);
sizeref = size(ScanRef,1);

%Motion estimation q0
u = motion.con.u;


x0 = u(1);
y0 = u(2);
yaw0 = u(3);

originalp = [x0 y0 yaw0];

p = originalp;
lasterror = [];
corr=[];
merr=0;


if DEBUG.lf_sog || DEBUG.all
    figure
    hold on
    axis equal
    hr = plot(ScanRef(:,1),ScanRef(:,2),'.k');
    hs = plot(NaN,NaN,'.b');
    title(['err = ' num2str(1/merr)]);
end

for i = 1:itMax
    
    %return if convergence is achieved
    if checkConv(lasterror, corr)
        break;
    end
    
    x = p(1);
    y = p(2);
    yaw = p(3);
    cosy = cos(yaw);
    siny = sin(yaw);
    
    % inialize grediant and hessian
    grad = zeros(1,3);
    H = zeros(3,3);
    dist_th = Opt.scanmatcher.Br(2);
    
    curscan = [];
    
    
    for j = 1:sizenew
        pn = ScanNew(j,1:2);
        
        %transform the point using the latest estimation
        v(1) = x + cosy*pn(1) - siny*pn(2);
        v(2) = y + siny*pn(1) + cosy*pn(2);
        
        %Jacobian of the transformation function having point pn and
        %translation x
        dv = [ 1, 0, -pn(1)*siny-cosy*pn(2);...
            0, 1, pn(1)*cosy-siny*pn(2)];
        
        curscan = [curscan [v(1); v(2) ] ];
        
        for k = 1:sizeref
            
            qn = ScanRef(k,1:2);
            
            %difference with mean
            dnm = (v - qn)';
            
            if sqrt(dnm(1)^2 + dnm(2)^2) > dist_th
                continue
            end
            
            expdnm = 2*exp(-dnm'*dnm);
            grad = grad + expdnm * (dnm'*dv);
            
            
            % Calculate the score
            merr = merr - exp( -(dnm(1)^2 + dnm(2)^2));
            
            % Calculate the Hessian of the likelihood function
            vH = [  -pn(1)*cosy+siny*pn(2);
                -pn(1)*siny-cosy*pn(2)];
            
            
            dnmdv1 = dnm'*dv(:,1);
            dnmdv2 = dnm'*dv(:,2);
            dnmdv3 = dnm'*dv(:,3);
            
            
            
            %Hessian of the function
            Hp =  [  ( (-2*dnmdv1*dnmdv1)+(dv(:,1)'*dv(:,1)) ),...% D² xx
                ( (-2*dnmdv2*dnmdv1)+(dv(:,1)'*dv(:,2)) ),... % D² xy
                ( (-2*dnmdv3*dnmdv1)+(dv(:,1)'*dv(:,3)) );... % D² xTHETA
                ...
                0,...% D² yx
                ( (-2*dnmdv2*dnmdv2)+(dv(:,2)'*dv(:,2)) ),... % D² yy
                ( (-2*dnmdv3*dnmdv2)+(dv(:,2)'*dv(:,3)) );... % D² yTHETA
                ...
                0,...% D² x THETA
                0,... % D² y THETA
                ( (-2*dnmdv3*dnmdv3)+( dnm'*vH + dv(:,3)'*dv(:,3) ) )...
                % D² THETA THETA. The second derivative is defined only in
                % this point
                ];
            
            %H is symmetric, let's fill the remaining part
            Hp(2,1) = Hp(1,2);
            Hp(3,1) = Hp(1,3);
            Hp(3,2) = Hp(2,3);
            
            H = H + expdnm*Hp;
        end
        
        
        
    end
    
    % Once we have the Hessian and Gradient of the function we calculate
    % the new translation
    invh = inv(H);
    if( sum(sum(isinf(invh))) == 0)
        dt = -invh*(grad');
    else
        % the Hessian inversion has failed
        break;
    end

    if DEBUG.lf_sog || DEBUG.all
        set(hs,'XData',curscan(1,:),'YData',curscan(2,:));
        drawnow
    end
    p = p + dt'; 
    p(3) = normAngle(p(3));
    
    
    % Maintain last errors to check convergence
    
    lasterror = [lasterror merr];
    corr = [corr; dt(1) dt(2) dt(3)];
    
    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
end


%Final result in quaternion
R = normAngle(p(3)-originalp(3) );

t = [p(1)-originalp(1) p(2)-originalp(2)];

NI = i;

end


