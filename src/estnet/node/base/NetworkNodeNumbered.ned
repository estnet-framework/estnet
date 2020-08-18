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
import estnet.node.errormodel.JammedPacketHandler;
import estnet.application.base.BasicApp;
import estnet.application.contract.IApp;
import estnet.application.contract.AppWrapper;
import estnet.application.positionbased.PositionBasedApp;
import estnet.application.common.AppHost;
import estnet.protocol.contract.IProtocolModule;
import estnet.radio.RadioHost;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.linklayer.contract.IWirelessInterface;

//
// Network Host for IProtocolModule implementations
//
module NetworkNodeNumbered like INetworkNodeNumbered
{
    parameters:
        @display("bgb=1202.4788,559.035");
        @networkNode;
        @labels(node,wireless-node);

        int nodeNo;                 // Node No
        int numRadios = default(1); // Number of Radios
        int numApps = default(1);	// Number of Apps

        string cpAppType = default("BasicRoutingTableApp"); // Type of Contact Plan App
        string radioType = default("RadioNic");             // Type of Radio

        string protocolModuleType; // Type of Protocol Module implementation
        string mobilityType;       // Type of Mobility

        string interfaceTableModule = default(absPath(".interfaceTable")); // path to interface table module
        **.interfaceTableModule = default(absPath(interfaceTableModule));


    gates:
        input radioIn[numRadios] @directIn;
        inout pppg[] @labels(PPPFrame-conn);
        inout ethg[] @labels(EtherFrame-conn);

    submodules:
        interfaceTable: InterfaceTable {
            parameters:
                @display("p=218,490;is=s");
        }
        appHost: AppHost {
            parameters:
                nodeNo = nodeNo;
                numApps = numApps;
                @display("p=855.1575,214.48125");
        }
        protocolModule: <protocolModuleType> like IProtocolModule {
            parameters:
                nodeNo = nodeNo;
                @display("p=718.16626,214.48125");
        }

        radioHost: RadioHost {
            parameters:
                nodeNo = nodeNo;
                numRadios = numRadios;
                @display("p=571.4888,214.48125");
        }

        jammedPacketHandler[numRadios]: JammedPacketHandler {
            radioIndex = index;
            @display("p=354.24,214.48125");
        }
        // AODV needs this to be called wlan
        wlan[numRadios]: <radioType> like IWirelessInterface {
            parameters:
                // every radio has the same mac address, so that the
                // receiver doesn't have to know which radio he needs
                // to talk to. That means we might need to deduplicate
                // frames when we receive them from more than one radio.
                mac.address = node_mac_address(nodeNo, 1);
                @display("p=90,216");
        }
        mobility: <mobilityType> like IMobility if mobilityType != "" {
            parameters:
                @display("p=114,490");
        }
        appWrapper[numApps]: AppWrapper {
            parameters:
                nodeNo = nodeNo;
                appId = index;
                @display("p=1039.1963,214.48125");
        }

    connections allowunconnected: // when numApps is 0
        for i=0..numApps-1 {
            appHost.toApp[i] --> appWrapper[i].wrapperIn;
            appWrapper[i].wrapperOut --> appHost.fromApp[i];
        }

        protocolModule.ifOut --> radioHost.upperLayerIn;
        protocolModule.ifIn <-- radioHost.upperLayerOut;
        for i=0..numRadios-1 {
            radioIn[i] --> wlan[i].radioIn;
            radioHost.lowerLayerIn[i] <-- jammedPacketHandler[i].upperLayerOut;
            jammedPacketHandler[i].lowerLayerIn <-- wlan[i].upperLayerOut;
            radioHost.lowerLayerOut[i] --> jammedPacketHandler[i].upperLayerIn;
            jammedPacketHandler[i].lowerLayerOut --> wlan[i].upperLayerIn;
        }
        appHost.lowerLayerIn <-- protocolModule.toAppHost;
        appHost.lowerLayerOut --> protocolModule.fromAppHost;
}
