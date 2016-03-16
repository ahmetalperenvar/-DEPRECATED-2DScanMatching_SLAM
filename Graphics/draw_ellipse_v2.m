function hp = draw_ellipse_v2(pos, cov, color,ObjectHandle)
%Two ways to call the function:
%
%[ObjectHandle] = draw_ellipse(pos, cov, color) Plots a ellipse with center at [pos],                                          
%                                               shape determined by [cov] and color 
%                                               by [color]. Returns figure handler
%  
%[]=draw_ellipse(pos, cov, color,ObjectHandle) Redefines shape and position of a  
%                                              previously defined ellipse called "handle". 

global PARAM;
%chi2=9.2103; %confian�a del 99%
%chi2=5.991; %confian�a del 95%
persistent CIRCLE

PARAM.chi2          =  chi2inv(0.99,2);
numero=size(cov,3);
p=[];


if isempty(CIRCLE) 
    tita = linspace(0, 2*pi,40); %es similar a usar :, genera un vector de 0 a 2pi amb 40 dades
    CIRCLE = [cos(tita); sin(tita)];
end

for i=1:numero
    if ~isnan(pos(i,1))
    [V,D]=eig(full(cov(1:2,1:2,i)));
    ejes=sqrt(PARAM.chi2*diag(D));
    P = (V*diag(ejes))*CIRCLE;
    p=[p [P(1,:)+pos(i,1);P(2,:)+pos(i,2)] [NaN; NaN]];
    end
end
    
if (nargin==3) && ~isempty(p)
    hp = line(p(1,:), p(2,:));
    set(hp,'Color', color);    
elseif nargin==4
    set(ObjectHandle, 'XData',p(1,:),'YData',p(2,:),'Color', color);
end




