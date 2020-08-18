//
// Copyright (C) 2020 Computer Science VII: Robotics and Telematics - 
// Julius-Maximilians-Universitaet Wuerzburg
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

package estnet.power.solarpanel;

import estnet.power.base.SatelliteSolarPanelBase;

//
// Solar Panel configuration for CubeSat side based on CESI CTJ-LC-SCA
//
simple CubeSatPanel_CESI extends SatelliteSolarPanelBase
{
    parameters:
        cellSize = 0.00265;  // panel size in m^2
        efficiency = 0.275;	// power retrieve efficiency of solar cells
        numSolarCells = default(2);		// number of cells of the panel, default is for 1U

        @class(SatelliteSolarPanelBase);
}
