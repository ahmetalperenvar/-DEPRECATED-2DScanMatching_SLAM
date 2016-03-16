% The association set is created using
% a different metric distance between a reference scan
% point ri , and its corresponding point ci . The distance
% function is defined as the norm of the transformation
% vector q which maps ci into ri . Being defined as q =
% x2 + y 2 + θ2 L2 , the new norm definition captures not
% only the points distance, but also the rotation (governed
% by the L parameter). Assuming θ small enough, the
% distance function is linearized around zero and the
% registration is solved using LMS over the linearized
% correspondence equation, to solve for qmin .

% J. Minguez, F. Lamiraux, and L. Montesano, “Metric-based scan match-
% ing algorithms for mobile robot displacement estimation,” in IEEE IN-
% TERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION,
% vol. 4. Citeseer, 2005, p. 3557.


function [R, t, NI] = MbICP(A, B, motion)

% In:
%           Ai: localCart[2xN], points cartesian 
%           Bi: localCart[2xN], points cartesian 
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations

global Opt DEBUG


it=1;


%Preprocess
ScA.cart = A.localCart(:,1:2)';
ScB.cart = B.localCart(:,1:2)';
ScA.polar = A.localPolar';
ScB.polar = B.localPolar';


itMax = Opt.scanmatcher.iterations;

%Motion estimation q0
u = motion.con.u;


x0 = u(1);
y0 = u(2);
yaw0 = u(3);

originalpose = [x0 y0 yaw0];


motion2D.estimation = originalpose;
motion2D.P = motion.state.P(1:3,1:3);

% initialization and some pre calculations
prec = Precompute(ScA);
lasterror = [];
corr =[];
y = motion2D.estimation(2);
yaw = motion2D.estimation(3);
x = motion2D.estimation(1);

opts =[];

if DEBUG.mbAssociation || DEBUG.all
    scrsz = get(0,'ScreenSize');
    opts.fighandle=figure('Position',scrsz,'Renderer','zbuffer','doublebuffer','on');
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    hold all
    opts.plot_r = plot(NaN,NaN,'.r','MarkerSize',6);
    opts.plot_n = plot(NaN,NaN,'.b','MarkerSize',6);
end



while it < itMax
    
    %% Apply q
    % return if convergence is achieved
    if checkConv(lasterror, corr)
        break;
    end
    
     % apply the motion to S New
    [ScBnf ScBf ScAf] = transPolarCartScan(ScB, ScA, yaw, [x y], 2,Opt.scanmatcher.Br(2));
    ScBnf.cart(3,:) = [];
    ScBf.cart(3,:) = [];
    
    if Opt.scanmatcher.projfilter
        BA = ScBf;
        AlA = ScAf;
    else
        BA = ScBnf;
        AlA = ScA;
    end
    
    %% Association
    
    %Compute the associated point with the Metric Based distance
    [~,assoc] = mbAssociation(BA, AlA, opts, motion2D,prec);
    

    %Use rejection rule?
    if ~isempty(Opt.scanmatcher.rejection_rule)
        assoc = Opt.scanmatcher.rejection_rule(assoc);
    end
    
    if size(assoc.ref,1) < 5
        break;
    end
    
    %% Registration
    %using Least Square
    [~,solution,merr] = registerMbICP(assoc,motion2D.estimation',10000,0);
    motion2D.estimation = solution;
    
    % update q
    yaw = normAngle(yaw + motion2D.estimation(3));
    x = x + motion2D.estimation(1);
    y = y + motion2D.estimation(2);    
    
    %% Error check
    lasterror = [lasterror 1/merr];
    corr = [corr; solution];
    
    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
    it=it+1;

end

%% Result
% Returns the minimizing Q = [t R]
R = normAngle(yaw-originalpose(3) );

t = [x y] - [originalpose(1:2)];

NI = it;

