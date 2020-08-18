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

package estnet.attitudecontroller;

import inet.power.base.EpEnergyConsumerBase;
//
// Implementation of a simple attitude controller for satellites which is able to performe
// various manouvers and calculates the consumped power for these.
//
module AttitudeController extends EpEnergyConsumerBase
{
    parameters:
        string angularAcceleration = default("0.12");
        // unit: rad/(s^2), if single value is given, it is the same for all three axis, if three values are given they are for (x,y,z).
  		// Example: "0.12" Based on a simpified model of a 1U CubeSat; "0.12, 0.024, 0.024" Based on a simpilfied model of a 3U CubeSat

        double maxPower @unit(mW) = default(600mW); 					// Peak power consumption
        double idlePowerConsumption @unit(W) = default(0W); 			// Idle power cosumption of the attitude controller

        string pathToEnergySource = default("^.energyModule.battery"); 	// sets the path to battery in EnergyModule as energy source
        string target = default("NIL");
        // Sets the initial target for the Attitude Controller; options: NIL, EARTH_CENTER, SUN, [Node Number], ECI coordinates

        double targetUpdateTime @unit(s) = default(60s); 				// Time the AttitudeController has to orientate to a specified target
        string pointingAxis = default("1,0,0"); 						// Axis which should point at the target (x,y,z)

        double recordingUpdateTime @unit(s) = default(0s);
        // extra recording timer for the pointing error; if set to 0 it is recorded with the targetUpdateTime

        @class(AttitudeController);

        @signal[accA](type=double);
        @signal[accB](type=double);
        @signal[accG](type=double);
        @signal[pointingError](type=double);
        @statistic[accA](title="AccelerationAlpha"; unit=rad/s^2; record=vector);
        @statistic[accB](title="AccelerationBeta"; unit=rad/s^2; record=vector);
        @statistic[accG](title="AccelerationGamma"; unit=rad/s^2; record=vector);
        @statistic[pointingError](title="PointingError"; unit=deg; record=vector);

}
