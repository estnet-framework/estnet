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

package estnet.node.tracking.target;
import estnet.node.tracking.contract.INodeTracking;

//
// Tracking based of specified targets ( SUN, NODE, NADIR,..)
//
simple TargetTracking like INodeTracking
{
    double updateInterval @unit(s) = default(0.1s); // minimum update intervall of tracking orientation
	string target = default("NIL"); // Sets the target: "EARTH_CENTER", "SUN", "NIL" or a node number
	bool enable = default(true);   // enable by default
	
	@signal[trackedTarget](type=unsigned long);
    @statistic[trackedTarget](title="Target Node Id"; record=vector,histogram);
}
