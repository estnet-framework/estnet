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

package estnet.power.base;

import inet.power.base.EpEnergyGeneratorBase;
import estnet.power.contract.ISolarPanel;
import estnet.mobility.base.OffsetMobility;

//
// Basic model of solar panel for satellites
// Describes power production on satellite
// based on the area of each cell, the sun's intensity,
// the efficiencies of the system and the power generation
// of the cells and the number of solar cells on the panel
//
// The orientation is modeled by an offset to the satellite frame
// with yaw-pitch-roll angles.d
//
module SatelliteSolarPanelBase extends EpEnergyGeneratorBase like ISolarPanel
{
    parameters:
        string energySinkModule = default("^.battery"); // module path of energy sink
        int checkInterval @unit(s) = default(10s); // interval at which energy production will be calculated
        double cellSize @unit(sqrm);  // cell size in m^2
        double sunIntensity @unit(Wpsqrm) = default(1367Wpsqrm); // @unit(W/m2)
        double efficiency;	// power retrieve efficiency of solar cells
        double systemLosses = default(0.9); // losses by transfering energy to the battery
        double absoluteSystemLoss = default(0); // absolute system loss in Watt
        int numSolarCells;		// number of cells of the panel
        double refractiveIndex = default(1);  	// the solar panel's surface's refractive index

        double yaw @unit(deg) = default(0deg);	// yaw angle; orientation to satellites base frame, z-axis is normal vector to panel plane
        double pitch @unit(deg) = default(0deg); // pitch angle; orientation to satellites base frame, z-axis is normal vector to panel plane
        double roll @unit(deg) = default(0deg); // roll angle; orientation to satellites base frame, z-axis is normal vector to panel plane
        @class(SatelliteSolarPanelBase);


 	submodules:
 	    mobility : OffsetMobility {
 	        parameters:
 	            yaw = yaw;
 	            pitch = pitch;
 	            roll = roll;
 	            parentMobility = default("^.^.^.networkHost.mobility");
 	    }
}
