function [Mapm Lmk Obs] = createMap(Rob,Sen,Tim)

% CREATEMAP Create an empty Map structure.
%   Map = CREATEMAP(Rob,Sen,Lmk,Opt) creates the structure Map from the
%   information contained in Rob, The map is saved as a list of point.
%   'occgrid' is cited but not working.
%Simone Zandara @ VICOROB TODO

global Opt

Map.type = Opt.map.type;

switch(Map.type) 
    
    case 'occgrid'
       Map.x = zeros(size(Rob.state.x)); %6D current pose (correspond to Robot.frame)
       Map.P = zeros(size(Rob.state.P));
       Map.grid = []; %zeros(Sen.par.dataPerScan* (Tim.firstFrame-Tim.lastFrame + 1),3 );
       Map.datasize = Sen.par.dataPerScan;
       Map.used = 1; %unused

       MM = struct('points',[]);
       
       steps = ceil((Tim.lastFrame - Tim.firstFrame) / Tim.step);
       
       Map.prev.gridlocal(1:steps) = MM;
       Map.prev.gridpolar(1:steps) = MM;
       Map.prev.gridlocalpolar(1:steps) = MM;
       Map.prev.gridcovm(1:steps) = MM;
       Map.prev.written(1:steps) = 0;
               
    otherwise

end   
Mapm = Map;
% Map.size = n;




% ========== End of function - Start GPL license ==========


%   # START GPL LICENSE

%---------------------------------------------------------------------
%
%   This file is part of SLAMTB, a SLAM toolbox for Matlab.
%
%   SLAMTB is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   SLAMTB is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>.
%
%---------------------------------------------------------------------

%   SLAMTB is Copyright 2007,2008,2009 
%   by Joan Sola, David Marquez and Jean Marie Codol @ LAAS-CNRS.
%   See on top of this file for its particular copyright.

%   # END GPL LICENSE

