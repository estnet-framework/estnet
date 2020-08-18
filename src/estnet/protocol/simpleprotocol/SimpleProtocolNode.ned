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

package estnet.protocol.simpleprotocol;


import estnet.node.base.NetworkNodeNumbered;
import estnet.node.manager.DummyNodeManager;

//
// Implementation of ~NetworkNodeNew for the simple
// protocol assembling the node with the correct
// protocol module and node manager.
//
module SimpleProtocolNode extends NetworkNodeNumbered
{
    parameters:
        protocolModuleType = "SimpleProtocolModule";
    submodules:
        nodeManager: DummyNodeManager {
            nodeNo = nodeNo;
        }
}
