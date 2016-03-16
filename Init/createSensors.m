function [Sen] = createSensors(Sensor)

% CREATESENSORS Create sensors structure array.
%   Sen = CREATESENSORS(Sensor) Creates the sensors simulator and
%   initializes error and type.

global Opt


for sen = 1:numel(Sensor)

    Si = Sensor{sen}; % input sensor structure

    % identification
    So.sen   = sen;
    So.id    = Si.id;
    So.name  = Si.name;
    So.type  = Si.type;
    So.robot = Si.robot;
    % transducer parameters
    
    So.frame.q = e2q(Si.orientationDegrees);
    So.frame.t = Si.position;
    
    So.raw.localCart = [];
    So.raw.localPolar = [];
    
    switch So.type


        case 'Multibeam'    
            So.par.maxWidth = Si.maxWidth;
            So.par.beamWidth = Si.beamWidth;
            So.par.nBeams = Si.nBeams;
            So.par.minRange = Si.minRange;
            So.par.maxRange = Si.maxRange;
            So.par.beamStd = Si.beamStd;
            So.par.dataPerScan = Si.dataPerScan;
            So.par.outliers = Si.outliers;
        case 'Multibeam2D'    
            So.par.maxWidth = Si.maxWidth;
            So.par.beamWidth = Si.beamWidth;
            So.par.nBeams = Si.nBeams;
            So.par.minRange = Si.minRange;
            So.par.maxRange = Si.maxRange;
            So.par.beamStd = Si.beamStd;
            So.par.dataPerScan = Si.dataPerScan;
            So.par.outliers = Si.outliers;
        case 'Singlebeam2D'    
            So.par.maxWidth = Si.maxWidth;
            So.par.beamWidth = Si.beamWidth;
            So.par.nBeams = Si.nBeams;
            So.par.minRange = Si.minRange;
            So.par.maxRange = Si.maxRange;
            So.par.beamStd = Si.beamStd;
            So.par.i = 0;
            So.par.dir = 1;
            So.par.dataPerScan = Si.dataPerScan;
            So.par.outliers = Si.outliers;
            
        case 'RealData'    
            So.par.maxWidth = Si.maxWidth;
            So.par.beamWidth = Si.beamWidth;
            So.par.nBeams = Si.nBeams;
            So.par.minRange = Si.minRange;
            So.par.maxRange = Si.maxRange;
            So.par.beamStd = Si.beamStd;
            So.par.i = 0;
            So.par.dir = 1;
            So.par.dataPerScan = Si.dataPerScan;
            So.par.outliers = Si.outliers;       
            
            load(Si.data,'beams');
            So.par.i = 0;
            So.data = beams;
            clear 'beams';
            
            
        otherwise
            error('??? Unknown sensor type ''%s''.',Si.type)

    end


    Sen(sen) = So; % output sensor structure
    


end




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

