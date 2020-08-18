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

package estnet.node.base;

//
// Common Base class for all communciating nodes
// in the simulation
//
module NodeBase
{
    parameters:
        string nodeContactManager = default("nodeManager"); // node manager module
        @signal[uplinkCoverage](type=long);
        @statistic[uplinkCoverage](title="uplink coverage"; source="uplinkCoverage"; record=vector,stats; interpolationmode=none);
        @signal[downLinkCoverage](type=long);
        @statistic[downLinkCoverage](title="downlink coverage"; source="downLinkCoverage"; record=vector,stats; interpolationmode=none);
        @class(NodeBase);
}
