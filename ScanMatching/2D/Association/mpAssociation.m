function [minr assoc index] = mpAssociation(snew, sref, Opt)
%MPASSOCIATION: Given a point and a reference set, find the best matching association
%based on the Matching Point rule as defined in

%Feng Lu and Evangelos Milios. 1997. Robot Pose Estimation in Unknown
%Environments by Matching 2D Range Scans. J. Intell. Robotics Syst. 18, 3
%(March 1997), 249-275

%It works in 2D, Z is kept fixed
% MODEL = Reference Scan
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
cart = sref.cart;
pa = snew.cart;

errtot = 0;
index = [];

% Orientation and distance threshold
Br = Opt.scanmatcher.Br;

global DEBUG
if DEBUG.cpAssociation || DEBUG.all

    set(Opt.plot_r,'XData',pset(:,1),'YData',pset(:,2));
    set(Opt.plot_n,'XData',pa(:,1),'YData',pa(:,2));
    drawnow
end


assoc = struct('new',[],'ref',[],'dist',[]);

for j = 1:size(pa,1)
    
    p = pa(j,1:2);
    [a l] = cart2pol(p(1),p(2));

    % initialization
    chosenp = [];        %best fitting range point
    minr = inf;          %minor range shift found
    m = size(pset,1);

    %We check all adjacent points for a possible association
    for i = 1:m
       
        ap = local(i,1);
        lp = local(i,2);
        
        pp = cart(i,1:2);
        w = abs(ap-a); 
        %check the angular shift fits the threshold
        if Br(1) > 0 && w > Br(1)
            continue;
        end
        
        % check the range and savfe the points having a closest range to
        % P
        mp1 = [ap lp]; 
        min_v = abs(l - lp);

        % minr stores the closest range we have found but the euclidean
        % distance between the two points still has to be minor than the
        % distance threshold
        if  ptsDistance(pp,p) < Br(2) && min_v < minr
            chosenp = mp1;
            minr = min_v;

        end
        
    end
    

    if(~isempty(chosenp))
        [pret1 pret2] = pol2cart(chosenp(1),chosenp(2));
        errtot = errtot + minr;
        assoc.new = [assoc.new; pa(j,1:3)];
        assoc.ref = [assoc.ref; [pret1 pret2] pa(j,3)];
        assoc.dist = [assoc.dist; minr];
%         if DEBUG.mpAssociation || DEBUG.all
%             plot( [p(1) pret1 ],[p(2) pret2 ], 'k' );
%         end
    end
    
    
end

% error as the sum of residuals
minr = mean(errtot);

end



