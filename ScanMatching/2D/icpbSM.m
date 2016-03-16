function [R t NI] = icpbSM(Ai, Bi, motion)
% ICMBSM: ICP based Scan Matcher. This function lets you perform a Scan
% Matching between two clouds of points defined inside Ai and Bi using an
% initial estimation MOTION. Opt stores the scan matcher parameters and the
% stage functions to be used. ICP is a 2 stage algorithm defined by an
% association step and a registration step.

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
opts = Opt;

Al.cart = Ai.localCart;
Al.polar = Ai.localPolar;

Bl.cart = Bi.localCart;
Bl.polar = Bi.localPolar;

Al.covm = Ai.covm;
Bl.covm = Bi.covm;

itMax = Opt.scanmatcher.iterations;

it=1;       %iteration control

sizeB = size(Bl.cart,1);

%Matrix initialization
B.cart = zeros(sizeB, 2);
B.polar = zeros(sizeB, 2);

err = zeros(sizeB, 1);
Br = opts.scanmatcher.Br;   % thresholds

if  DEBUG.all
    scrsz = get(0,'ScreenSize');
    opts.fighandle=figure('Position',scrsz,'Renderer','zbuffer','doublebuffer','on');
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    hold all
    opts.plot_r = plot(NaN,NaN,'.r','MarkerSize',6);
    opts.plot_n = plot(NaN,NaN,'.b','MarkerSize',6);
end


%Motion estimation q0

u = motion.con.u;

x0 = u(1);
y0 = u(2);
yaw0 = u(3);

x=x0;
y=y0;
yaw=yaw0;

dirPoints = 1;

originalodo = [cos(u(2))*u(1) sin(u(2))*u(1) u(3)];
motion2D.estimation = originalodo;
motion2D.P = motion.state.P(1:3,1:3);

par1=[];
par2=[];

% depending on the algorithm we have the scan ordered differently

if strcmp(func2str(Opt.scanmatcher.associate),'mahaAssociation')
    par1 =  motion2D;
    par1.estimation = [0 0 0];
    
    Bl.cart = Bl.cart';
    Al.cart = Al.cart(:,1:2)';
    Bl.polar = Bl.polar';
    Al.polar = Al.polar';
    dirPoints=2;
    [B Bf Af aixs bixs] = transPolarCartScan(Bl, Al, yaw, [x y], dirPoints,Opt.scanmatcher.Br(2));
    %B = Bl;
    B.cart(3,:) = [];
    Bf.cart(3,:) = [];
    
    B.P = Bi.covm;
    Al.P = Ai.covm;
    
    
    Bf.P = Bi.covm;
    Bf.P(:,:,bixs) = [];
    Af.P = Ai.covm;
    Af.P(:,:,aixs) = [];
    
elseif strcmp(func2str(Opt.scanmatcher.associate),'mbAssociation')
    
    Bl.cart = Bl.cart';
    Al.cart = Al.cart(:,1:2)';
    Bl.polar = Bl.polar';
    Al.polar = Al.polar';
    
    
    
    par1 = motion2D;
    par2 = Precompute(Al);
    
    dirPoints=2;
    
    [B Bf Af] = transPolarCartScan(Bl, Al, yaw, [x y], dirPoints,opts.scanmatcher.Br(2));
    B.cart(3,:) = [];
    Bf.cart(3,:) = [];
    
else
    
    [B Bf Af] = transPolarCartScan(Bl, Al, yaw, [x y], dirPoints,opts.scanmatcher.Br(2));
end


Cnf = B;
Cf = Bf;


lasterror = [];
corr=[];

while it < itMax
    
    %% Apply q
    % reeturn if convergence is achieved
    if checkConv(lasterror,corr)
        break;
    end
    
    
    %% Association
    
    if isempty(Opt.scanmatcher.associate)
        err('No association function defined')
    end
    
    if Opt.scanmatcher.projfilter
        CA = Cf;
        AlA = Af;
    else
        CA = Cnf;
        AlA = Al;
    end
    
    
    if ~isempty(par1) && ~isempty(par2)
        [merr assp i1] = Opt.scanmatcher.associate(CA,AlA,Opt,par1,par2);
    elseif ~isempty(par1)
        [merr assp i1] = Opt.scanmatcher.associate(CA,AlA,Opt,par1);
    else
        [merr assp i1] = Opt.scanmatcher.associate(CA,AlA,Opt);
    end
    
    
    opts.scanmatcher.Br(1) = max(Br(1)*exp(-alpha*it),0.1);
    
    if isempty(Opt.scanmatcher.register)
        err('No registration function defined')
    end
    
    
    %Use rejection rule?
    if ~isempty(Opt.scanmatcher.rejection_rule)
        assp = Opt.scanmatcher.rejection_rule(assp);
    end
    
    
    
    %% Registration
    if size(assp.ref,1) < 5
        break;
    end
    
    if size(assp.new,2) == 2
        assp.new(:,3) = Ai.localCart(1,3);
        assp.ref(:,3) = Ai.localCart(1,3);
    end
    
    [R t] = Opt.scanmatcher.register(assp.new', assp.ref');
    
    % update q
    yawR = R2e(R);
    
    yaw = normAngle(yaw + yawR(3));
    x = x + t(1);
    y = y + t(2);
    
    % apply the motion to S New
    [Cnf Cf Af] = transPolarCartScan(Bl, Al, yaw, [x y], dirPoints,Opt.scanmatcher.Br(2));
    
    if strcmp(func2str(Opt.scanmatcher.associate),'mahaAssociation')
        Cnf.P = Bi.covm;
        Cnf.cart(3,:) = [];
        Cf.P = Bi.covm;
        Cf.cart(3,:) = [];
    end
    
    it=it+1;
    
    % error estimation update
    lastyaw = yawR(3);
    lasterror = [lasterror 1/merr]
    corr = [corr; t(1) t(2) lastyaw]
    
    if size(lasterror,2) > Opt.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
end
%% Result
% Returns the minimizing Q = [t R]
R =  normAngle(yaw-yaw0);
t = [x y] - [x0 y0];


NI = it;

