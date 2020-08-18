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

import inet.power.base.EpEnergyConsumerBase;
import estnet.power.contract.IConsumerStateHandler;

//
// Essential subsystem which is active all the time
// Therefore, a consumption is given, as well as a efficiency
// Example useage:
//	*.sat[*].energyModule.consumer[0].constantConsumers[0].powerConsumption = 0.014W
//	*.sat[*].energyModule.consumer[0].constantConsumers[0].submoduleName = "OBC"
//
simple ConstantConsumer extends EpEnergyConsumerBase like IConsumerStateHandler
{
    parameters:
        string submoduleName = default("CPU"); // name of this module
        string energySourceModule = default("^.^.battery"); // module path of energy source
        double powerConsumption @unit(W); // constant consumption of this module
        double efficiency = default(0.855); // efficiency of consumer
        @class(ConstantConsumer);
}





