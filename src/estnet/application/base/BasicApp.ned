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

package estnet.application.base;

import estnet.application.contract.IApp;

// Basic App with multiple nodes as destination
simple BasicApp like IApp
{
    parameters:
        string destinationNodes = default("1"); 				// representation of all the destination node numbers
        int nodeNo;												// id of containing node
        int appId;												// id of app
        int destAppId = default(appId);							// id of the destination app
        bool sending = default(false);							// Whether the app is generating data
        double startTime @unit(s) = default(0s);				// When the app starts generating data
        double stopTime @unit(s) = default(-1s);				// When the app stops generating data
        volatile double sendInterval @unit(s) = default(60s);	// Interval with which data is generated
        volatile int payloadSize @unit(B) = default(100B);		// The payload size of the generated packets
        bool printNoPackets = default(true);					// Wether the app should print the number of packets it sent and received
        bool printReceivedPacketsNo = default(false);			// Whether the app should print packets it received
        bool printMissingPacketsNo = default(false);			// Whether the app should print packets it didn't receive
        bool printSentPacketsNo = default(false);				// Whether the app should print packets it send
        @class(BasicApp);

        @signal[sentPk](type=cPacket);
        @signal[rcvdPk](type=cPacket);
        @statistic[rcvdPk](title="packets received"; source=rcvdPk; record=count,"vector(constantOne)","vector(constantMinusOne)","sum(packetBytes)","vector(packetBytes)","vector(negativePacketBytes)", "vector(pktSequenceNumber)"; interpolationmode=none);
        @statistic[sentPk](title="packets sent"; source=sentPk; record=count,"vector(constantOne)","sum(packetBytes)","vector(packetBytes)", "vector(pktSequenceNumber)"; interpolationmode=none);
        @statistic[sentThroughput](title="sentThroughput"; unit=bps; source="throughput(sentPk)"; record=histogram,stats,vector);
        @statistic[rcvdThroughput](title="rcvdThroughput"; unit=bps; source="throughput(rcvdPk)"; record=histogram,stats,vector);
        @statistic[rcvdPkLifetime](title="received packet lifetime"; source="messageAge(rcvdPk)"; unit=s; record=histogram,stats,vector; interpolationmode=none);
        @statistic[rcvdPkNumHops](title="received packet number of hops"; source="pktNumHops(rcvdPk)"; record=histogram,stats,vector; interpolationmode=none);
    gates:
        input appIn;
        output appOut;
}

