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

package estnet.common.queue;

import inet.queueing.queue.DropTailQueue;


//
// A ~DropTailQueueShared with additional statistics being collected
//
simple DropTailQueueExtraStatistics extends DropTailQueue {
  parameters:
    @signal[rcvdPk](type=cPacket);
    @signal[enqueuePk](type=cPacket);
    @signal[dequeuePk](type=cPacket);
    @signal[dropPkByQueue](type=cPacket);
    @signal[queueingTime](type=simtime_t; unit=s);
    @signal[queueLength](type=long);
    @statistic[rcvdPk](title="received packets"; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
    @statistic[enqueuePk](title="received packets"; record=count,"sum(packetBytes)","vector(packetBytes)","vector(basicRadioFrameSequenceNumber)","vector(basicRadioFrameSrcNodeNo)"; interpolationmode=none);
    @statistic[dequeuePk](title="received packets"; record=count,"sum(packetBytes)","vector(packetBytes)","vector(basicRadioFrameSequenceNumber)","vector(basicRadioFrameSrcNodeNo)"; interpolationmode=none);
    @statistic[dropPk](title="dropped packets"; source=dropPkByQueue; record=count,"sum(packetBytes)","vector(packetBytes)","vector(basicRadioFrameSequenceNumber)","vector(basicRadioFrameSrcNodeNo)"; interpolationmode=none);
    @statistic[queueingTime](title="queueing time"; record=histogram,vector; interpolationmode=none);
    @statistic[queueLength](title="queue length"; record=max,timeavg,vector; interpolationmode=sample-hold);
}
