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

import estnet.node.contract.INetworkNodeNumbered;
import estnet.application.contract.IApp;
import estnet.application.contract.IAppForwarder;
import estnet.radio.contract.IRadioForwarder;
import estnet.radio.RadioNic;
import estnet.radio.RadioHost;
import inet.queueing.contract.IPacketQueue;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;

///
/// Network Host for ION
/// @deprecated in favor of ~IProtocolModule
///
module NodeIndirect like INetworkNodeNumbered
{
    parameters:
        @display("bgb=,448");
        @networkNode;
        @labels(node,wireless-node);

        int nodeNo;
        int numRadios = default(1);

        string appType = default("BasicApp");
        string appForwarderType;
        string radioForwarderType;
        string mobilityType;
        string queueType = default("DropTailQueueShared");  ///< Type of Queue

        string interfaceTableModule = default(absPath(".interfaceTable"));
        **.interfaceTableModule = default(absPath(interfaceTableModule));

    gates:
        input radioIn[numRadios] @directIn;
        inout pppg[] @labels(PPPFrame-conn);
        inout ethg[] @labels(EtherFrame-conn);

    submodules:
        app: <appType> like IApp {
            parameters:
                nodeNo = nodeNo;
        }
        appForwarder: <appForwarderType> like IAppForwarder {
            parameters:
                nodeNo = nodeNo;
        }
        radioForwarder: <radioForwarderType> like IRadioForwarder {
            parameters:
                nodeNo = nodeNo;
        }
        queue: <queueType> like IPacketQueue {
            parameters:
                @display("p=23,125;q=l2queue");
        }
        radioHost: RadioHost {
            parameters:
                nodeNo = nodeNo;
                numRadios = numRadios;
        }
        // AODV needs this to be called wlan
        wlan[numRadios]: RadioNic {
            parameters:
                // every radio has the same mac address, so that the
                // receiver doesn't have to know which radio he needs
                // to talk to. That means we might need to deduplicate
                // frames when we receive them from more than one radio.
                mac.address = node_mac_address(nodeNo, 1);
        }
        mobility: <mobilityType> like IMobility if mobilityType != "";
        interfaceTable: InterfaceTable {
            parameters:
                @display("p=53,300;is=s");
        }
    connections:
        app.appIn <-- appForwarder.toApp;
        app.appOut --> appForwarder.fromApp;
        radioForwarder.ifOut --> queue.in;
        radioForwarder.ifIn <-- radioHost.upperLayerOut;
        queue.out --> radioHost.upperLayerIn;
        for i=0..numRadios-1 {
            radioIn[i] --> wlan[i].radioIn;
            radioHost.lowerLayerIn[i] <-- wlan[i].upperLayerOut;
            radioHost.lowerLayerOut[i] --> wlan[i].upperLayerIn;
        }
}
