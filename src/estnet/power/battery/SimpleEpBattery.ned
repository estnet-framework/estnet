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

package estnet.power.battery;
import inet.power.storage.SimpleEpEnergyStorage;

//
// Describes energy storage module
// modifies inets SimpleEpEnergyStorage
//
simple SimpleEpBattery extends SimpleEpEnergyStorage
{
    nominalCapacity = default(35100J); // the capacity of the battery
    initialCapacity = default(0.8*nominalCapacity); // the state of charge at simulation start
    @class(SimpleEpBattery);
}
