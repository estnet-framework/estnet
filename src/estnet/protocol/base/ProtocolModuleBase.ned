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

package estnet.protocol.base;

import estnet.protocol.contract.IProtocolModule;

//
// Shared base class for ~IProtocolModule implementations
// providing some common interface code with the Omnet++
// environment
//
module ProtocolModuleBase like IProtocolModule
{
    parameters:
        int nodeNo;                                 // Node No
        bool dropAlreadySeenFrames = default(true); // Whether frames should be deduplicated, to prevent routing loops
        bool fakeSentRoutingTable = default(true);  // Whether a routing table is given to the protocol at initialization


        @signal[rcvdPacketFromHL](type=cPacket);
        @signal[sentPacketToHL](type=cPacket);
        @signal[incomingFrame](type=cPacket);
        @signal[incomingDiscardedFrame](type=cPacket);
        @signal[forwardedFrame](type=cPacket);
        @signal[outgoingFrame](type=cPacket);
        @signal[outgoingDiscardedFrame](type=cPacket);
        @signal[droppedPk](type=cPacket);
        @signal[bufferedPk](type=cPacket);
        @signal[bufferedFrame](type=cPacket);
        @signal[bufferingTime](type=simtime_t; unit=s);
        @signal[bufferLength](type=unsigned long);

        @statistic[rcvdPacketFromHL](title="packets received from higher layer"; source=rcvdPacketFromHL; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[sentPacketToHL](title="packets sent to higher layer"; source=sentPacketToHL; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[incomingFrame](title="frames received"; source=incomingFrame; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[incomingDiscardedFrame](title="incoming frames dropped"; source=incomingDiscardedFrame; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[outgoingFrame](title="frames sent"; source=outgoingFrame; record=count,"sum(packetBytes)","vector(packetBytes)","vector(pktNextHopNodeNo)"; interpolationmode=none);

        @statistic[outgoingDiscardedFrame](title="outgoing frames dropped"; source=outgoingDiscardedFrame; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[forwardedFrame](title="frames forwarded"; source=forwardedFrame; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[incomingFrameLifetime](title="received frame lifetime"; source="messageAge(incomingFrame)"; unit=s; record=stats,vector; interpolationmode=none);

        @statistic[droppedPk](title="dropped packet"; source=droppedPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[bufferedPk](title="buffered packet"; source=bufferedPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);

        @statistic[bufferedFrame](title="buffered frame"; source=bufferedFrame; record=count,"sum(packetBytes)","vector(packetBytes)","vector(pktNextHopNodeNo)"; interpolationmode=none);

        @statistic[rcvdThroughput](title="throughput"; unit=bps; source="throughput(incomingFrame)"; record=histogram,stats,vector);
        //@statistic[sentThroughput](title="throughput"; unit=bps; source="throughput(outgoingFrame)"; record=histogram,stats,vector);

        @statistic[bufferingTime](title="buffering time"; record=histogram,vector; interpolationmode=none);

        @statistic[bufferLength](title="buffer length"; record=max,timeavg,vector; interpolationmode=sample-hold);
    gates:
        input fromAppHost;
        output toAppHost;
        input ifIn;
        output ifOut;
}
