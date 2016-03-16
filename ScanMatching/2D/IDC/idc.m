% It works similarly to the original ICP,
% but solving the association at two different stages. First,
% the Closest Point Rule is used to compute the corre-
% spondence set needed for solving for the translation
% registering through the Unit Quaternions [5]. Next, the
% Matching Range Rule is used to associate points having
% the closest range when represented in polar coordinates.
% It is worth noting that polar range is invariant to rotation.
% Next, the new association set is used to register (again
% with the Unit Quaternions [5]) solving for the rotation.

%Feng Lu and Evangelos Milios. 1997. Robot Pose Estimation in Unknown
%Environments by Matching 2D Range Scans. J. Intell. Robotics Syst. 18, 3
%(March 1997), 249-275.


function [R, t, NI] = idc(Ai, Bi, motion)

% In:
%           Ai: localCart[2xN], points cartesian, localPolar[2xN], points
%           polar
%           Bi: localCart[2xN], points cartesian, localPolar[2xN], points
%           polar
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations


global DEBUG Opt
opts = Opt;

% we need both cartesian and polar
Al.cart = Ai.localCart;
Al.polar = Ai.localPolar;
Bl.cart = Bi.localCart;
Bl.polar = Bi.localPolar;

itMax = opts.scanmatcher.iterations;
Br = opts.scanmatcher.Br;   % thresholds
alpha = 0.05;   % angular threshold decreasing factor
it=1;       %iteration control

sizeB = size(Bl.cart,1);

% Matrix initialization
B.cart = zeros(sizeB, 2);
B.polar = zeros(sizeB, 2);


%Motion estimation q0
u = motion.con.u;
pose(3) = 0;

x0 = u(1);
y0 = u(2);
yaw0 = u(3);

x=x0;
y=y0;
yaw=yaw0;

lasterror = [];
corr=[];

% Format the two scans we are going to use
B.cart(:,1:2) = Bl.cart(:,1:2);
B.polar(:,1:2) = Bl.polar(:,1:2);

A.cart(:,1:2) = Al.cart(:,1:2);
A.polar(:,1:2) = Al.polar(:,1:2);

if DEBUG.cpAssociation || DEBUG.all
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
        break
    end
    
    % apply the motion to S New
    [Bnf Bf Af] = transPolarCartScan(B, Al, yaw, [x y], 1,opts.scanmatcher.Br(2));
    Bnf.cart(:,3) =  pose(3); 
    Bf.cart(:,3) =  pose(3);
    
    if opts.scanmatcher.projfilter
        BA = Bf;
        AlA = Af;
    else
        BA = Bnf;
        AlA = Al;
    end
    
    %% Association
    
    %Compute the associated point with the Closest Point rule
    [merr assp ] = cpAssociation(BA,AlA, opts);
    
    
    %Compute the associated point with the Matching Range rule
    [~, assp2 ] = mpAssociation(BA,AlA, opts);
    
    % Decrement the angular threshold
    opts.scanmatcher.Br(1) = max(Br(1)*exp(-alpha*it),0.1);
    
    %Use rejection rule?
    if ~isempty(opts.scanmatcher.rejection_rule)
        assp = opts.scanmatcher.rejection_rule(assp);
        assp2 = opts.scanmatcher.rejection_rule(assp2);
    end
    
    %% Registration
    
    if size(assp.new,1) < 5 || size(assp2.new,1) < 5
        break;
    end
    
    
    %Registrate the scan, Use rotation from IMRP rule if selected
    %[BB ASS] = filterAssMatrix(B.cart, assp,i1);
    [~, t] = regist_besl(assp.new', assp.ref');
    
    %[BB ASS] = filterAssMatrix(B.cart, assp2.ref,i2);
    [R] = regist_besl(assp2.new', assp2.ref');
    
    
    % update q
    yawR = R2e(R);
    yaw = yaw + yawR(3);
    x = x + t(1);
    y = y + t(2);
    
    %Error estimation, keep a vector of the last error estimation
    it=it+1;
    lasterror = [lasterror merr/10];
    corr = [corr; t(1)/10 t(2)/10 yaw/10];
    
    % error estimation update
    if size(lasterror,2) > opts.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
    
end

%% Result
% Returns the minimizing Q = [t R]
R = normAngle(yaw-yaw0);
t = [x y] - [x0 y0];
NI = it;
