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

package estnet.contactplan.common;

//
// Reads a contact plan and provides
// all modules in the simulation
// access to all or just active contacts.
//
simple ContactPlanManager
{
    parameters:
        string contactPlanFile = default("");           // the contact plan file to read, skipped if empty string
        string routingTableCreator = default("hops");   // optimizes routing table for min hops if "hops" or min time if "time"
        bool shouldWaitForLimboFlush = default(false);  // whether to wait for protocols to flush their limbo @deprecated
        @signal[connectionCount](type=long);
        @signal[cpSimtime](type=long);
        @signal[seenSatelliteCount](type=long);
        @statistic[connectionCount](title="Connection count"; source="connectionCount"; record=vector,stats; interpolationmode=none);
        @statistic[seenSatelliteCount](title="seen Satellite Count"; source="seenSatelliteCount"; record=vector,stats; interpolationmode=none);
        @statistic[cpSimtime](title="sim time contact plan"; source="cpSimtime"; record=vector,stats; interpolationmode=none);
}
