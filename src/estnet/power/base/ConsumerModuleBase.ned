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

import inet.power.base.EpEnergyConsumerBase;
import estnet.power.contract.IConsumerModule;

//
// Base module for all consumer modules
//
module ConsumerModuleBase extends EpEnergyConsumerBase like IConsumerModule
{
    string energySourceModule = default("^.^.battery"); // module path to battery
    bool disableInEclipse = default(false);				// decides whether consumer is disabled in eclipse phases<
    @class(ConsumerModuleBase);
}
