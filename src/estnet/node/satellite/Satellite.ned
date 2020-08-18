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

package estnet.node.satellite;

import estnet.node.contract.INetworkNodeNumbered;
import estnet.visualizer.OsgNode;
import estnet.attitudecontroller.AttitudeController;
import estnet.power.EnergyModule;
import estnet.node.base.NodeBase;
import estnet.node.errormodel.NodeFailureModel;


//
// Represents physical satellite with osgNode
// for 3D visualization and newtworkHost for
// communication
//
module Satellite extends NodeBase
{
    parameters:
        int nodeNo;                     // Node no
        string hostType = default("estnet.protocol.simpleprotocol.SimpleProtocolNode");  // Network Host Type
        string label = default("");     // Node Label
        bool errorModelEnable = default(false); // enables the error model
        bool energyModelEnable = default(false); // enables the energy model
        bool alignmentCheckEnable = default(false); // only starts sending messages if it is correctly allignet
        bool contactTrackingEnable = default(false);	// tracks nodes based on the contact plan
        double contactTrackingUpdateTime @unit(s) = default(60.0s);	// update intervall for contact tracking
        @display("i=device/satellite;bgb=400,400;t=$label");
        @class(Satellite);

    submodules:
        networkHost: <hostType> like INetworkNodeNumbered {
            @display("p=100, 100");
            nodeNo = nodeNo;
            mobilityType = default("SatMobility");
        }
        // only working if OsgEarthScene is in network and SatMobility is Mobility-Type
        osgNode: OsgNode {
            @display("p=100, 300");
        }
        attitudeController: AttitudeController {
            @display("p=300,100");
        }
        nodeFailureModel: NodeFailureModel if errorModelEnable != false {
            @display("p=100,200");
        }
        energyModule: EnergyModule if energyModelEnable != false {
            @display("p=300,200");
        }
}
