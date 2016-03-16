function SimWorld = createWorld(World)

% CREATEWORLD  Create a world for simulation.
%   SIMLMK = CREATEWORLD(World) defines limits and mesh for the world.


if isempty(World.points)
    World.points = zeros(3,0);
end
if isempty(World.segments)
    World.segments = zeros(6,0);
end

PN = size(World.points,2); % number of points in the simulated world
SN = size(World.segments,2); % number of segments

SimWorld.points.id      = (1:PN);
SimWorld.points.coord   = World.points;
SimWorld.segments.id    = PN+(1:SN); 
SimWorld.segments.coord = World.segments;
SimWorld.surface = World.surface;
SimWorld.lims.xMin      = World.xMin;
SimWorld.lims.xMax      = World.xMax;
SimWorld.lims.yMin      = World.yMin;
SimWorld.lims.yMax      = World.yMax;
SimWorld.lims.zMin      = World.zMin;
SimWorld.lims.zMax      = World.zMax;
SimWorld.dims.l         = SimWorld.lims.xMax - SimWorld.lims.xMin; % playground dimensions
SimWorld.dims.w         = SimWorld.lims.yMax - SimWorld.lims.yMin;
SimWorld.dims.h         = SimWorld.lims.zMax - SimWorld.lims.zMin;
SimWorld.center.xMean   = (SimWorld.lims.xMax + SimWorld.lims.xMin)/2; % center fo playground
SimWorld.center.yMean   = (SimWorld.lims.yMax + SimWorld.lims.yMin)/2;
SimWorld.center.zMean   = (SimWorld.lims.zMax + SimWorld.lims.zMin)/2;



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

