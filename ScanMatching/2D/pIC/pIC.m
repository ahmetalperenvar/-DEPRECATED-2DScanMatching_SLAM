% Uses a probabilistic framework to keep
% into consideration the motion and sensor noise. Asso-
% ciation uses de Closest Point Rule but defined over
% the Mahalanobis distance of the points instead of the
% Euclidean counterpart. The original pIC algorithm uses
% a virtual point as the correspondence one. For a given
% reference point ri , its associated virtual point ai is placed
% in the probabilistic weight average of the statistically
% compatible points of the new scan (those which satisfy
% the Individual Compatibility test). pIC registration is
% done as in the M bICP but using the

% L. Montesano, J. Minguez, and L. Montano, “Probabilistic scan match-
% ing for motion estimation in unstructured environments,” in Intelligent
% Robots and Systems, 2005.(IROS 2005). 2005 IEEE/RSJ International
% Conference on. IEEE, 2005, pp. 3499–3504.


function [R, t, NI] = pIC(A, B, motion)

% In:
%           Ai: localCart[2xN], points cartesian, covm[2x2xN] points
%           uncertainty
%           Bi: localCart[2xN], points cartesian, covm[2x2xN] points
%           uncertainty
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%                                       rotational and orientation
%                   STATE.P[3x3] scan uncertainty
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations

global Opt DEBUG
opts = Opt;

t = [];
it=0;

% Format the two scans we are going to use
ScA.cart = A.localCart(:,1:2)';
ScB.cart = B.localCart(:,1:2)';

ScA.polar = A.localPolar(:,1:2)';
ScB.polar = B.localPolar(:,1:2)';

itMax = opts.scanmatcher.iterations;

ScA.P = A.covm;
ScB.P = B.covm;

%Motion estimation q0
u = motion.con.u;

x0 = u(1);
y0 = u(2);
yaw0 = u(3);

originalodo = [x0 y0 yaw0];

motion2D.estimation = originalodo;
solution = originalodo';
motion2D.P = motion.state.P([1 2 4],[1 2 4]); %eye(3)*0.005;

% error convergence init
lasterror = [];
corr = [];


if DEBUG.mahaAssociation || DEBUG.all
    scrsz = get(0,'ScreenSize');
    opts.fighandle=figure('Position',scrsz,'Renderer','zbuffer','doublebuffer','on');
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    hold all
    opts.plot_r = plot(NaN,NaN,'.r','MarkerSize',6);
    opts.plot_n = plot(NaN,NaN,'.b','MarkerSize',6);
    opts.plot_a = plot(NaN,NaN,'-m','LineWidth',1);
end

while  it < itMax
    
    % Check convergence on previous error
    if checkConv(lasterror, corr)
        break
    end
    
    %% Association
    
    %Compute the associated point with the Mahalanobis Distance
    [merr assoc] = mahaAssociation(ScB, ScA, opts, motion2D);
    
    if isempty(assoc)
        break;
    end
    
    %Use rejection rule?
    if ~isempty(opts.scanmatcher.rejection_rule)
        [assoc ixs] = opts.scanmatcher.rejection_rule(assoc);
        assoc.oldn(:,ixs) =  [];
        assoc.Pe(:,:,ixs) =  [];
        assoc.Jq(:,:,ixs) =  [];
    end
    
    if size(assoc.ref,1) < 5
        break;
    end
    
    if DEBUG.all
        displayAssociations(assoc,opts.plot_a);
        
    end
    %% Registration
    %using Least Square
    [~, solution] = pICRegistration(assoc,motion2D,10000,0);
    
    motion2D.estimation = solution';
    
    
    %% Error check
    lasterror = [lasterror 1/merr];
    corr = [corr; solution'];
    
    if (sum(diag(motion2D.P)) > 0.05)
        
        motion2D.P = motion2D.P*0.8;
        
    end
    
    if size(lasterror,2) > opts.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
    it=it+1;
end


%% Result
% Returns the minimizing Q = [t R]
solution = solution' - originalodo;
t(1:2) = [solution(1) solution(2) ];
R = solution(3);
NI = it;