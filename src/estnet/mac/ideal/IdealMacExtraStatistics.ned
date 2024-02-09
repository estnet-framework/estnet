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

package estnet.mac.ideal;

import inet.linklayer.contract.IMacProtocol;
import inet.linklayer.acking.AckingMac;
//
// An ~IdealMacQueue with additional statistics being collected
//
module IdealMacExtraStatistics extends AckingMac like IMacProtocol
{
    parameters:
        // Signal dropPkNotForUs does not exist anymore in INET4, maybe use unsigned long numDroppedNotForUs in EtherMacBase.h instead
        //@signal[dropPkNotForUs](type=BasicApplicationPacket);
        @signal[linkBroken](type=);
        @statistic[passedUpPk](title="packets passed to higher layer"; source=packetSentToUpper; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[sentDownPk](title="packets sent to lower layer"; source=packetSentToLower; record=count,"sum(packetBytes)","vector(packetBytes)","vector(macFrameSrcNodeNo)","vector(macFrameDestNodeNo)"; interpolationmode=none);
        @statistic[rcvdPkFromHL](title="packets received from higher layer"; source=packetReceivedFromUpper; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[rcvdPkFromLL](title="packets received from lower layer"; source=packetReceivedFromLower; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        //@statistic[droppedPkNotForUs](title="packets dropped/not for us"; source=dropPkNotForUs; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[lostPkNoReceiver](title="packets lost/no receiver"; source=linkBroken; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[packetDropIncorrectlyReceivedButForUs](title="packet drop: incorrectly received but for us"; source=packetDroppedDueToBitErrorOrCollision(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
}
