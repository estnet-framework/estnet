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

import estnet.mobility.contract.IExtendedMobility;


//
// The StaticTerrestrialMobility module provides position
// calculation (earth rotation) for surface-fixed communication
// devices (e.g. ground stations) and 3D vizualisation in OSG
//
module StaticTerrestrialMobility like IExtendedMobility
{
    parameters:
        double lat @unit(deg) = default(0.0deg);            // latitude of node in degress
        double lon @unit(deg) = default(0.0deg);            // longitude of node in degress
        double alt @unit(m) = default(0.0m); 				// altitude of node in meters
        bool enableKinematics = default(false);				// if enabled the ground station's orientation will be limited by the maximum raotation rates
        double maxAzimuthRate @unit(radps) = default(0.1083307812radps);		// maximum rotation rate around the azimuth angle
        double maxElevationRate @unit(radps) = default(0.04688944259radps);	// maximum rotation rate around the elevation angle

        // Switch to enable/disable self-triggered updates of the satMobility module
        bool enableSelfTrigger = default(false);
        // Time interval for self-triggered updates, if enabled
        double selfTriggerTimeIv @unit(s) = default(10s);

        @signal[azimuth](type=double);
        @signal[elevation](type=double);
        @signal[azimuthError](type=double);
        @signal[elevationError](type=double);
        @signal[positionUpdateX](type=double); 
        @signal[positionUpdateY](type=double); 
        @signal[positionUpdateZ](type=double); 
        @signal[velocityUpdateX](type=double);
        @signal[velocityUpdateY](type=double);
        @signal[velocityUpdateZ](type=double);
        @statistic[azimuth](title="Azimuth"; unit=deg; record=vector);
        @statistic[elevation](title="Elevation"; unit=deg; record=vector);
        @statistic[azimuthError](title="AzimuthError"; unit=deg; record=vector);
        @statistic[elevationError](title="ElevationError"; unit=deg; record=vector);
        @statistic[eciPositionX](title="ECI position x"; source=positionUpdateX; record=vector; interpolationmode=none);
        @statistic[eciPositionY](title="ECI position y"; source=positionUpdateY; record=vector; interpolationmode=none);
        @statistic[eciPositionZ](title="ECI position z"; source=positionUpdateZ; record=vector; interpolationmode=none);
        @statistic[eciVelocityX](title="ECI velocity x"; source=velocityUpdateX; record=vector; interpolationmode=none);
        @statistic[eciVelocityY](title="ECI velocity y"; source=velocityUpdateY; record=vector; interpolationmode=none);
        @statistic[eciVelocityZ](title="ECI velocity z"; source=velocityUpdateZ; record=vector; interpolationmode=none);
        @display("i=device/receiverdish");
        @class(StaticTerrestrialMobility);
}
