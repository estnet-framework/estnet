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

package estnet.mobility.base;

import estnet.mobility.contract.IExtendedMobility;

simple OffsetMobility like IExtendedMobility 
{
    parameters:
        string parentMobility = default("^.^.^.^.mobility");
        double yaw @unit(deg) = default(0.0); 	// rotation around the z-axis of the node it's contained in
        double pitch @unit(deg) = default(0.0); // rotation around the x-axis of the node it's contained in
        double roll @unit(deg) = default(0.0);	// rotation around the y-axis of the node it's contained in
        double offsetX @unit(m) = default(0.0); // offset along the x-axis of the node it's contained in
        double offsetY @unit(m) = default(0.0); // offset along the y-axis of the node it's contained in
        double offsetZ @unit(m) = default(0.0); // offset along the z-axis of the node it's contained in
        // Switch to enable/disable self-triggered updates of the satMobility module
        bool enableSelfTrigger = default(false);
        // Time interval for self-triggered updates, if enabled
        double selfTriggerTimeIv @unit(s) = default(10s);
        @class(OffsetMobility);
}
