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

package estnet.node.groundstation;

import estnet.node.base.NodeBase;
import estnet.node.contract.INetworkNodeNumbered;
import estnet.visualizer.OsgNode;
import estnet.node.tracking.contract.INodeTracking;

// Represents a ground station node
// It contains a networkhost with mobility module, tracking 
// module and it is represented with OSG.
module GroundStation extends NodeBase
{
    parameters:
        int nodeNo;	//node number
        string hostType = default("estnet.protocol.simpleprotocol.SimpleProtocolNode"); // protocol module
        string trackingType = default("estnet.node.tracking.contactplanbased.ContactPlanBasedNodeTracking"); // tracking mocule
        string label = default(""); // label displayed in the view
        bool fillUpApps = default(true); // parameter to decide whether to automatically create pseudo receicing only basic apps for maximum app number of all satellites

        bool internetConnection = default(true); // whether it can talk to any other internet connected ground station
        @display("i=device/receiverdish;bgb=200,400;t=$label");
        @class(GroundStation);

    submodules:
        networkHost: <hostType> like INetworkNodeNumbered {
            @display("p=100, 100");
            nodeNo = nodeNo;
            mobilityType = default("StaticTerrestrialMobility");
        }
        // only working if OsgEarthScene is in network and SatMobility or StaticTerrestrialMobility is Mobility-Type
        osgNode: OsgNode {
            @display("p=100, 300");
        }
        // tracking module for satellite tracking
        nodeTracking: <trackingType> like INodeTracking;
}

