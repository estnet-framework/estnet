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

package estnet.mobility.terrestrial;

import inet.mobility.contract.IMobility;


//
// The StaticTerrestrialMobility module provides position
// calculation (earth rotation) for surface-fixed communication
// devices (e.g. ground stations) and 3D vizualisation in OSG
//
module StaticTerrestrialMobility like IMobility
{
    parameters:
        double lat @unit(deg) = default(0.0deg);            // latitude of node in degress
        double lon @unit(deg) = default(0.0deg);            // longitude of node in degress
        double alt @unit(m) = default(0.0m); 				// altitude of node in meters
        bool enableKinematics = default(false);				// if enabled the ground station's orientation will be limited by the maximum raotation rates
        double maxAzimuthRate = default(0.1083307812);		// maximum rotation rate around the azimuth angle
        double maxElevationRate = default(0.04688944259);	// maximum rotation rate around the elevation angle

        @signal[azimuth](type=double);
        @signal[elevation](type=double);
        @signal[azimuthError](type=double);
        @signal[elevationError](type=double);
        @statistic[azimuth](title="Azimuth"; unit=deg; record=vector);
        @statistic[elevation](title="Elevation"; unit=deg; record=vector);
        @statistic[azimuthError](title="AzimuthError"; unit=deg; record=vector);
        @statistic[elevationError](title="ElevationError"; unit=deg; record=vector);

        @display("i=device/receiverdish");
        @class(StaticTerrestrialMobility);
}
