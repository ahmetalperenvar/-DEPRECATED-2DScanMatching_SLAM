function [mind assoc index] = cpAssociation(snew, sref, Opt)
%CPASSOCIATION. Given a point and a reference set, find the best matching association
%based on the Closest Point rule, search over the N points in pset the on
%with minor Euclidean distance.

% In:    
%         SNEW.cart: Nx2 points
%         SREF.cart: Nx2 points
%         OPT: options and plot data
% Out:   
%         MIND mean error
%         ASSOC association object
%         INDEX index of the points in SREF associated with the points in SNEW

pset = sref.cart;
pa = snew.cart;

global DEBUG
if DEBUG.cpAssociation || DEBUG.all

    set(Opt.plot_r,'XData',pset(:,1),'YData',pset(:,2));
    set(Opt.plot_n,'XData',pa(:,1),'YData',pa(:,2));
    drawnow
end

index = [];
errtot = 0;

assoc = struct('new',[],'ref',[],'dist',[]);

% Loop through all the points in the new scan 
for j = 1:size(pa,1)
    
    p = pa(j,1:2);
    a = pa(j,3);
    
    
    infd = inf;
    mind = 0;
    pf = [];
    indexf = -1;
    
    for i = 1:size(pset,1)
        p2 = pset(i,1:2); 
        
        % classic Euler Distance
        d = sqrt( (p(1)-p2(1))^2 + (p(2)-p2(2))^2 );
        
        %we select the point with minor distance and only considering those
        %points with distance less than a certain threshold Br(2). The last
        %part of the IF is a switch to disable such threshold
        if d < infd && d < (Opt.scanmatcher.Br(2)+(~Opt.scanmatcher.distancethreshold*10000))
            infd = d;
            mind = infd;
            pf =  p2;
            indexf = i;
        end
        
    end
    
    % only generates the output if we found at least 1 association
    if indexf > 0
        assoc.new = [assoc.new; pa(j,1:3)];
        assoc.ref = [assoc.ref; pf pa(j,3)];
        assoc.dist = [assoc.dist; mind];
        errtot = errtot + mind;

%         if DEBUG.cpAssociation || DEBUG.all
%             plot( [p(1) pf(1) ],[p(2) pf(2) ], 'm' )
%         end
        
    end
    
    index = [index; indexf];
end

% The error is the sum of the residuals
mind = mean(errtot);

end
