function [ out ] = checkConv( err, t, minerr )
%CHECKCONV Given an input of values consevutive representing error function outputs, returns
%whether the function is converging. The convergence tolerance is given as
%input

global Opt;
out = 0;

if nargin==2
    minerr = 1;
end

if size(err,2)  > 0

%     if minerr && err(end) >= Opt.scanmatcher.chival
%         return;
%     end
    
    dt = sqrt(t(:,1).^2 + t(:,2).^2);
    dth = t(:,3);

    if size(err,2) >= Opt.scanmatcher.niterconv &&...
            (std(err) < Opt.scanmatcher.convalue ||...
            std(dt) < Opt.scanmatcher.convalue*10 && std(dth) < Opt.scanmatcher.convalue*10)
        out = 1;
    end
    
end

end

