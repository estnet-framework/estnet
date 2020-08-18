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
// Energy Consumer Module
// with constant on/off cycle
// Example usage:
//	*.sat[*].energyModule.consumer[0].IMA[0].handlerType1 = "ConstantDutyCycleConsumer"
//	*.sat[*].energyModule.consumer[0].IMA[0].stateHandler1.powerConsumption = 2640mW
//	*.sat[*].energyModule.consumer[0].IMA[0].stateHandler1.offPowerConsumption = 181.5mW
//	*.sat[*].energyModule.consumer[0].IMA[0].stateHandler1.onTime = 0.1s
//	*.sat[*].energyModule.consumer[0].IMA[0].stateHandler1.offTime = 19.85s
//
simple ConstantDutyCycleConsumer extends ConsumerStateHandler
{
    parameters:
        double onTime @unit(s); // time this module is active
        double offTime @unit(s); // time this module is inactive
        double powerConsumption @unit(W);	// consumption when activated
        double offPowerConsumption @unit(W) = default(0W); // consumption when inactive
        double efficiency = default(0.855);	// efficiency of consumer
        @class(ConstantDutyCycleConsumer);
}




