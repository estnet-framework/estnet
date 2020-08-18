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

package estnet.node.errormodel;

//
// module that decides whether the packets are passed or if they 
// have to be thrown away as the satellite is in jamming range of
// a jamming station or if the satellite is in a failure state
//
simple JammedPacketHandler
{
    parameters:
        int radioIndex;	// index if the belonging radio that can be jammed
        @signal[jammedPacketCount](type=long);
        @statistic[jammedPacketCount](title="jammed packet count"; source=jammedPacketCount; record=last,vector);
        @signal[lostPacketCount](type=long);
        @statistic[lostPacketCount](title="lost packet count, to node failure"; source=lostPacketCount; record=last,vector);

    gates:
        input upperLayerIn;
        input lowerLayerIn;
        output upperLayerOut;
        output lowerLayerOut;


}
