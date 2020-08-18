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

package estnet.radio;

import estnet.radio.contract.IRadioHost;


//
// Decides which radio a packet needs to be sent with.
// Deduplicates received packets over multiple radios.
//
module RadioHost like IRadioHost
{
    parameters:
        int numRadios = default(1);             // Number of radios
        int nodeNo;                             // Node No
        string queueModule = default(".RH_NIC_queue[0]"); // Path from the network host to the queue module
        @class(RadioHost);
    gates:
        // these two gates talk to the layer above the radio
        input upperLayerIn;
        output upperLayerOut;
        // this gate is used to forward messages to the upper layer in the C++ module
        input forwardToUpperLayer @directIn;
        // these two gates communicate with the real radio underneath
        output lowerLayerOut[numRadios];
        input lowerLayerIn[numRadios];
    connections allowunconnected: // when numRadios is 0
        forwardToUpperLayer --> upperLayerOut;
}
