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

package estnet.power.consumer.type;

import estnet.power.base.ConsumerStateHandler;

//
// The system is active when the satellite is over a target.(lat/long)
// The range is defined by the (min.) elevation 
// Example usage:
//	*.sat[*].energyModule.consumer[0].IMA[*].stateHandler1.powerConsumption = 485mW
//	*.sat[*].energyModule.consumer[0].IMA[*].stateHandler1.offPowerConsumption = 350mW
//	*.sat[*].energyModule.consumer[0].IMA[*].stateHandler1.longitude = 9.980713deg 
//	*.sat[*].energyModule.consumer[0].IMA[*].stateHandler1.latitude = 49.786844deg
//
module TargetTrackingConsumer extends ConsumerStateHandler
{
    parameters:
        string pathToEnergyModule = default("^.^.^.^.energyModule"); // path to energy module
        double powerConsumption @unit(W);	// consumption when activated
        double offPowerConsumption @unit(W) = default(0W); // consumption when inactive
        double efficiency = default(0.855); // efficiency of consumer
        double minElevation @unit(deg) = default(10deg); // minumum elevation to target for activation
        double longitude @unit(deg); // longitude of target point
        double latitude @unit(deg);  // latitude of target point
        @class(TargetTrackingConsumer);
}
