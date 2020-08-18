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

package estnet.contactplan;

import estnet.node.contract.INetworkNodeNumbered;
import estnet.radio.RadioHost;
import estnet.contactplan.ContactPlanCreatingApp;
import inet.networklayer.common.InterfaceTable;
import inet.queueing.contract.IPacketQueue;
import inet.linklayer.contract.IWirelessInterface;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;


//
// Network Host used to generate
// contact plans.
// Assembles the special modules
// together.
//
module ContactPlanCreatingNode like INetworkNodeNumbered
{
    parameters:
        @display("bgb=,448");
        @networkNode;
        @labels(node,wireless-node);

        int nodeNo;
        int numRadios = default(1);
        string mobilityType = default(numRadios > 0 ? "StationaryMobility" : "");
        string radioType = default("RadioNic");
        string queueType = default("inet.queueing.queue.DropTailQueue");

        string interfaceTableModule = default(absPath(".interfaceTable"));
        **.interfaceTableModule = default(absPath(interfaceTableModule));

    gates:
        input radioIn[numRadios] @directIn;
        inout pppg[] @labels(PPPFrame-conn);
        inout ethg[] @labels(EtherFrame-conn);

    submodules:
        interfaceTable: InterfaceTable {
            parameters:
                @display("p=53,300;is=s");
        }
        app: ContactPlanCreatingApp {
            parameters:
                nodeNo = nodeNo;
                @display("p=216,53");
        }
        queue: <queueType> like IPacketQueue {
            parameters:
                @display("p=351,145;q=l2queue");
        }

        radioHost: RadioHost {
            parameters:
                nodeNo = nodeNo;
                numRadios = numRadios;
                @display("p=216,236");
        }
        // AODV needs this to be called wlan0
        wlan[numRadios]: <radioType> like IWirelessInterface {
            parameters:
                @display("p=216,406,row,60;q=queue");
                // every radio has the same mac address, so that the
                // receiver doesn't have to know which radio he needs
                // to talk to. That means we might need to deduplicate
                // frames when we receive them from more than one radio.
                mac.address = node_mac_address(nodeNo, 1);
        }
        mobility: <mobilityType> like IMobility if mobilityType != "";
    connections:
        app.lowerLayerOut --> queue.in;
        app.lowerLayerIn <-- radioHost.upperLayerOut;
        queue.out --> radioHost.upperLayerIn;
        for i=0..numRadios-1 {
            radioIn[i] --> wlan[i].radioIn;
            radioHost.lowerLayerIn[i] <-- wlan[i].upperLayerOut;
            radioHost.lowerLayerOut[i] --> wlan[i].upperLayerIn;
        }
}
