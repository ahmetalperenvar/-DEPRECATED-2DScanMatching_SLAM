%% Random world generator
%% SYNTAX  POLYGON = wall_generator(0.3);figure; axis equal; plot(POLYGON(1,:),POLYGON(2,:))

classdef  wall_generator
    
    properties
        POLYGON=[];
        TRAJ = [];
    end
    
    methods
        
        %constructor
        function obj = wall_generator(varargin)
            if nargin == 0
                
                
                obj.POLYGON=wall_generator.random_world(0.1);
            else
                if varargin{1}>0
                    obj.POLYGON=wall_generator.random_world(varargin{1});
                else
                    obj.POLYGON=wall_generator.random_world_unstruct(varargin{1});
                end
                
                if nargin>1 && varargin{2}==1
                    disp(obj);
                    traj_i = ginput;
                    if ( size(traj_i,1)>0)
                        obj.TRAJ = wall_generator.calc_traj(traj_i');
                    end
                end
            end
            
            
        end
        
        
        function disp(obj)
            figure; plot(obj.POLYGON(1,:),obj.POLYGON(2,:),'LineWidth',3); axis equal;
            
        end
        
        
        
    end
    
    methods (Static)
        
        function [traj] = calc_traj(traj)
            for x = 2:size(traj,2)
                
                
                dif = traj(:,x) - traj(:,x-1);
                
                if( sum(dif(1:2)) ~= 0 )
                    traj(6,x) = atan2(dif(2,:), dif(1,:));
                else
                    traj(6,x) = traj(6,x-1);
                end
            end
        end
        
        function [pol] = draw_ellipse(x0,y0,a,b,theta) % angular positions of vertices
            t = linspace(0, 2*pi, 30);
            
            for i = 1:length(x0)
                % pre-compute rotation angles (given in degrees)
                cot = cosd(theta(i));
                sit = sind(theta(i));
                
                % compute position of points used to draw current ellipse
                xt = x0(i) + a(i) * cos(t) * cot - b(i) * sin(t) * sit;
                yt = y0(i) + a(i) * cos(t) * sit + b(i) * sin(t) * cot;
                
            end
            
            [x_e,y_e] = poly2cw(xt, yt);
            pol=[x_e;y_e];
            
        end
        
        
        function [pol] = draw_rect(x0,y0,a,b,theta) % angular positions of vertices
            
            x_rect(1,1) = x0-a/2; x_rect(1,2) = x0+a/2; x_rect(1,3) = x0+a/2; x_rect(1,4) = x0-a/2; x_rect(1,5) = x0-a/2;
            x_rect(2,1) = y0+b/2; x_rect(2,2) = y0+b/2; x_rect(2,3) = y0-b/2; x_rect(2,4) = y0-b/2; x_rect(2,5) = y0+b/2;
            
            T=[cos(theta) -sin(theta) 0; sin(theta)  cos(theta) 0;   0 0 1];
            
            for l = 1:size(x_rect,2)
                rotated_rect(:,l) = T(1:2,1:2)*[x_rect(1,l);x_rect(2,l)];
            end
            [x_e,y_e] = poly2cw(rotated_rect(1,:),rotated_rect(2,:));
            
            
            
            pol=[x_e;y_e];
        end
        
        %Generator
        function polygon = random_world(ell_sigma)
            
            npoly = round(rand*10)+4;
            polygon = [];
            
            
            for n = 1:npoly
                type = rand;
                add=rand;
                if add>0.8
                    add='-';
                else
                    add='+';
                end
                this_pol = [];
                
                x0_r = rand*5+1;
                y0_r = rand*5+1;
                a_r = rand*3+1;
                b_r = rand*3+1;
                theta_r = 0; %rand*(pi/2);
                
                if type > ell_sigma %rect
                    this_pol = wall_generator.draw_rect(x0_r,y0_r,a_r,b_r,theta_r);
                else %ellipse
                    this_pol = wall_generator.draw_ellipse(x0_r,y0_r,a_r/2,b_r/2,theta_r);
                end
                
                if ~isempty(polygon)
                    [polygon_x,polygon_y] = polybool(add,polygon(1,:),polygon(2,:),this_pol(1,:),this_pol(2,:));
                    polygon=[polygon_x;polygon_y];
                else
                    polygon = this_pol;
                end
            end
            
        end
        
        %Generator
        function polygon = random_world_unstruct(ell_sigma)

            last_p = [(rand*3)-1;(rand*3)-1];
            last_angle = 0;
            angle = pi*2;
            polygon = last_p;
            init_p=last_p;
            
            for l = 1:1 %round(rand*2)+2
                npoly = 40;
                for n = 1:npoly
                    angle = rand*(pi*2);
                    while angle>last_angle+pi/1.5 || angle < last_angle-pi/1.5
                        angle = rand*(pi*2);
                    end
                    last_angle = angle;
                    d = rand*2;
                    
                    polygon_x = last_p(1) + d*cos(angle);
                    polygon_y = last_p(2) + d*sin(angle);
                    
                    polygon=[polygon [polygon_x;polygon_y] ];
                    
                    last_p = [polygon_x; polygon_y];
                    
                end
                
                polygon=[polygon [NaN;NaN] ];
                
                last_p = init_p + [(rand*8)-4;(rand*8)-4];
                init_p = last_p;
                
                last_angle = 0;
                angle = pi*2;
                polygon = [polygon last_p];
                
            end
            
        end
        
    end
    
end





