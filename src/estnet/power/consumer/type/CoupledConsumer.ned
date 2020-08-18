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
// The system is activated if other
// specific subsystems are active.
// Example usage:
//	*.sat[*].energyModule.consumer[0].NAV[*].stateHandler1.powerConsumption = 0.165W
//	*.sat[*].energyModule.consumer[0].NAV[*].stateHandler1.pathToCoupledConsumer = ".^.^.OCS[0]"
//
module CoupledConsumer extends ConsumerStateHandler
{
    parameters:
        double powerConsumption @unit(W);	// consumption when activated
        double offPowerConsumption @unit(W) = default(0W); // consumption when inactive
        double efficiency = default(0.855);	// efficiency of consumer
        string pathToCoupledConsumer; // module path to consumer on which this module is coupled
        @class(CoupledConsumer);
}
