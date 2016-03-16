function [ R, t ] = register_martinez( A, B )
% REGISTER_MARTINEZ Closed form registration to solve the least square
% minimization over two set of 2D associated points. Closed form inspired
% by: 

% Mobile robot motion estimation by 2D scan matching with genetic and 
% iterative closest point algorithms
% Martinez, J. L. Gonzalez, J. Morales, J. Mandow, A. Garcia-Cerezo, A. J.
% JOURNAL OF FIELD ROBOTICS
% 2006, VOL 23; NUMBER 1, pages 21-34
A = A';
B = B';

sizen = size(A,1);

sumb = sum(B);
sumbx = sumb(:,1);
sumby = sumb(:,2);

suma = sum(A);
sumax = suma(:,1);
sumay = suma(:,2);

sumabx = sum(A(:,1).*B(:,1));
sumaby = sum(A(:,2).*B(:,2));

sumabyx = sum(A(:,2).*B(:,1));
sumabxy = sum(A(:,1).*B(:,2));

yaw = atan( (sumay*sumbx + sizen*(sumabxy - sumabyx) - sumby*sumax ) / ...
             ( sizen*(sumabx + sumaby) - sumax*sumbx - sumby*sumay) );
         
x = (sumbx - cos(yaw)*sumax + sin(yaw)*sumay)/sizen;
y = (sumby - sin(yaw)*sumax - cos(yaw)*sumay)/sizen;

t = [x y 0];
R = e2R([0 0 yaw]);
end

