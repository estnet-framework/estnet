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

package estnet.radio.apsk;

import inet.physicallayer.apskradio.packetlevel.ApskScalarRadio;

//
// An ~APSKScalarRadio with additional statistics being collected
//
module APSKScalarRadioExtraStatistics extends ApskScalarRadio
{
    parameters:
        @statistic[sentThroughput](title="sentThroughput"; unit=bps; source="throughput(packetReceivedFromUpper)"; record=histogram,stats,vector);
        @statistic[rcvdThroughput](title="rcvdThroughput"; unit=bps; source="throughput(packetSentToUpper)"; record=histogram,stats,vector);
        @statistic[minSnir](title="Min SNIR"; source=minimumSnir(packetSentToUpper); record=histogram,stats,vector);
        @statistic[packetErrorRate](title="Packet error rate"; source=packetErrorRate(packetSentToUpper); record=histogram,stats,vector);
        @statistic[bitErrorRate](title="Bit error rate"; source=bitErrorRate(packetSentToUpper); record=histogram,stats,vector);
        @statistic[symbolErrorRate](title="Symbol error rate"; source=symbolErrorRate(packetSentToUpper); record=histogram,stats,vector);
        @class(APSKScalarRadioExtraStatistics);

}
