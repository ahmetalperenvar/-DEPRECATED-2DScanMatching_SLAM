% Calculate intersection between a single beam and the walls defined by the
% world. The beam is selected using an index nbeam. Every sonar has
% S.maxWidth coverage, divided into S.nBeams, S.beamWidth wide.

function [simulatedScan, realPoints] = intersectPoint2DWall(TrajScan,nbeam,Xmax,Ymax,S,noise)



sonarRange = S.maxRange;
sonarWidth = S.maxWidth;
sonarAngle = 1;

simulatedScan=[-1 -1];
realPoints=[0 0];
% for i = 1:nBeams %360 is even number WARNING: this works for 360DEG!
%     
     x=TrajScan(1);
     y=TrajScan(2);
     yaw=TrajScan(3);
% 
%     %calculate intersection between y = mx + q (beam line) with the 4 wall
%     %lines
        angle =  -sonarWidth / 2;
        angle = deg2rad(angle + nbeam*sonarAngle)+yaw; %yaw is in rad CHECK IF - or +
        m = tan(angle);
        q = y-m*x;
    
        intersection=zeros(4,2);		
        noise = stdErr(0,noise);
        
        
        intersection(1,1)=0;            intersection(1,2)=q;            % x=0
        intersection(2,1)=(Ymax-q)/m;   intersection(2,2)=Ymax;         % x=Ymax
        intersection(3,1)=Xmax;         intersection(3,2)=m*Xmax+q;     % y=Xmax
        intersection(4,1)=-q/m;         intersection(4,2)=0;            % y=0
        intersection = addRow(intersection,[noise noise]);
        
    for j=1:4
        %intersection is outside the pool/room/environment
        if (intersection(j,1)<0 || intersection(j,1)>Xmax || intersection(j,2)<0 || intersection(j,2)>Ymax)
            continue;
        end

        distance=sqrt((intersection(j,1)-x)^2+(intersection(j,2)-y)^2);
        xSign=cos(angle);
        ySign=sin(angle);

        %intersection on direction given by angle
        if( (intersection(j,1)-x)*xSign>=0 && (intersection(j,2)-y)*ySign>=0)
            if(distance>sonarRange) 
                simulatedScan=[-1 -1];
            else
                simulatedScan = [ angle-yaw  distance ];
                realPoints = intersection(j,:);           
            end                
        end
    end   
% end
% indexDelete = (simulatedScan == 0);
% realPoints([indexDelete;indexDelete])=[];
% realPoints = reshape(realPoints,2,[]);
% 
% [r,c,v] = find(simulatedScan);
% simulatedScan = [sonarAngle*c;v];

% if DEBUG.plotscans
%     figure(nu_scan)
%     subplot(2,3,1)
%     pp=zeros(2,size(simulatedScan,2));
%     for i=1:size(simulatedScan,2)
%         pp(1,i)=simulatedScan(2,i)*cos(deg2rad(simulatedScan(1,i)));
%         pp(2,i)=simulatedScan(2,i)*sin(deg2rad(simulatedScan(1,i)));
%     end
%     
%     subplot(2,3,1)
%     axis equal
%     plot(pp(1,:),pp(2,:),'r.','DisplayName','simulatedScan');
%     title(['Simulated Scan ' num2str(nu_scan)])
%         
%     subplot(2,3,4)
%     axis equal
%     plot(realPoints(1,:),realPoints(2,:),'r*','DisplayName','realPoints');
%     hold all
%     plot(TrajScan(1),TrajScan(2),'b.','DisplayName','trajectory');
%     title(['Real Map ' num2str(nu_scan)])
% end


