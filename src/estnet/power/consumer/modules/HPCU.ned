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

package estnet.power.consumer.modules;

import estnet.power.base.ConsumerModuleBase;
import estnet.power.contract.IConsumerStateHandler;


//
// Module for consumption of the High Performance Computing Unit Module
// The module's consumption is modeled by two state handlers
//
module HPCU extends ConsumerModuleBase
{
    parameters:
        int numHandler = default(2);
        string handlerType1 = default("VaryingDutyCycleConsumer"); 
        string handlerType2 = default("CoupledConsumer");
    submodules:
        stateHandler1: <handlerType1> like IConsumerStateHandler if handlerType1 != "";
        stateHandler2: <handlerType2> like IConsumerStateHandler if handlerType2 != "";
}
