function [mind assoc index] = p2lAssociation(snew, sref, Opt)
%P2LASSOCIATION Point to line association. Tries to associate every point
%of SNEW to one in SREF. A point in SREF is chosen using a point to line
%metric, ie calculating the orthogonal distance beteween a point in SNEW
%and a line formed by consecutive points in the model SREF

% In:
%         SNEW: cart[Nx2 points],polar[Nx2 points]
%         SREF: cart[Nx2 points],polar[Nx2 points]
%         OPT: options and plot data
% Out:
%         MIND mean error
%         ASSOC association object
%         INDEX index of the points in SREF associated with the points in
%         SNEW


pset = sref.cart;
local = sref.polar;

pa = snew.cart;
Br = Opt.scanmatcher.Br;

global DEBUG
if DEBUG.cpAssociation || DEBUG.all
    
    set(Opt.plot_r,'XData',pset(:,1),'YData',pset(:,2));
    set(Opt.plot_n,'XData',pa(:,1),'YData',pa(:,2));
    drawnow
end

pret = [];
index = [];
errtot = 0;

assoc = struct('new',[],'ref',[],'dist',[]);

% Cycle all the points in S ne
for j = 1:size(pa,1)
    
    p = pa(j,1:2);
    a =  snew.polar(j,1);
    
    
    infd = inf;
    indexf = -1;
    pf = [-1 -1];
    
    for i = 1:size(pset,1)-1
        
        ap = local(i,1);
        w = abs(ap-a);
        
        %Maximum rotation check
        if Br(1) > 0 && w > Br(1)
            continue;
        end
        
        %Adjacent points in the reference scan
        p2 = pset(i,1:2);
        p3 = pset(i+1,1:2);
        
        %interpolate the points and find the segment distance from p
        [d P] = pt2lineDistance(p(1:2),p2,p3);
        
        %we select the point with minor distance but has
        %still euclidean distance lower than a threshold
        if d < Br(2) && d < infd
            infd = d;
            pf = P;
            indexf = i;
        end
        
    end
    pret = [pret; pf];
    index = [index; indexf];
  
    if indexf > 0
        assoc.new = [assoc.new; pa(j,1:3)];
        assoc.ref = [assoc.ref; pf pa(j,3)];
        assoc.dist = [assoc.dist; infd];
        errtot = errtot + infd;
%         if DEBUG.p2lAssociation || DEBUG.all
%             plot( [p(1) pf(1) ],[p(2) pf(2) ], 'm' )
%         end
        
    end
    
end

% error as the sum of residuals
mind = mean(errtot);

end

