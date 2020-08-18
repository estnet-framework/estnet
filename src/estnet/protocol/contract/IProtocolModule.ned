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

package estnet.protocol.contract;

//
// Shared interface to plug DTN protocols
// into the simulation
//
moduleinterface IProtocolModule
{
    parameters:
        int nodeNo; // the Node no

        @signal[rcvdPacketFromHL](type=cPacket);
        @signal[sentPacketToHL](type=cPacket);
        @signal[incomingFrame](type=cPacket);
        @signal[incomingDiscardedFrame](type=cPacket);
        @signal[forwardedFrame](type=cPacket);
        @signal[outgoingFrame](type=cPacket);
        @signal[outgoingDiscardedFrame](type=cPacket);
        @signal[bufferingTime](type=simtime_t; unit=s);
        @signal[bufferLength](type=unsigned long);
    gates:
        //connects protocol module with apps
        input fromAppHost;
        output toAppHost;
        // AODV requires these gate names
        input ifIn; // fromRadio
        output ifOut; // toRadio
}
