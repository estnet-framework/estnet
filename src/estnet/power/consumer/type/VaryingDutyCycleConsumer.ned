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
// Energy Consumer Module with stochastical varying dutycycle
// variation is set by std param
// Example usage:
//	*.sat[*].energyModule.consumer[0].HPCU[*].stateHandler1.MTA = 20s
// 	*.sat[*].energyModule.consumer[0].HPCU[*].stateHandler1.MTR = 180s
//	*.sat[*].energyModule.consumer[0].HPCU[*].stateHandler1.powerConsumption = 1W
//	*.sat[*].energyModule.consumer[0].HPCU[*].stateHandler1.std = 10
//
module VaryingDutyCycleConsumer extends ConsumerStateHandler
{
    parameters:
        double MTA @unit(s);	// mean time of activation
        double MTR @unit(s);	// mean time to reactivation
        double std;				// standard deviation
        double powerConsumption @unit(W);	// consumption when activated
        double offPowerConsumption @unit(W) = default(0W); // consumption when inactive
        double efficiency = default(0.855);	// efficiency of consumer
        @class(VaryingDutyCycleConsumer);
}
