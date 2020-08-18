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

package estnet.application.common;

//
// The AppHost manages all apps and types of apps of a node.
// It forwards packages from the apps and distributes received packages to the different apps.
//
simple AppHost
{
    parameters:
        @display("bgb=,448");

        int nodeNo;                 // Node id
        int numApps; 				// Number of apps

        @signal[sentPk](type=cPacket);
        @signal[rcvdPk](type=cPacket);
        @statistic[rcvdPk](title="packets received"; source=rcvdPk; record=count,"vector(constantOne)","vector(constantMinusOne)","sum(packetBytes)","vector(packetBytes)","vector(negativePacketBytes)","vector(pktSrcNodeNo)","vector(pktDestNodeNo)"; interpolationmode=none);
        @statistic[sentPk](title="packets sent"; source=sentPk; record=count,"vector(constantOne)","sum(packetBytes)","vector(packetBytes)","vector(pktSrcNodeNo)","vector(pktDestNodeNo)"; interpolationmode=none);

    gates:
        input fromApp[numApps+1];	// from apps
        input lowerLayerIn;			// from protocol module
        output toApp[numApps+1];	// to apps
        output lowerLayerOut;		// to protocol module
}
