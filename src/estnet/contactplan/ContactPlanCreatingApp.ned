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

package estnet.contactplan;

//
// An app used to generate data during
// contact plan creation.
// Strictly not needed anymore, as the
// ~ContactPlanCreatingRadioMedium fakes
// its own traffic, but kept for debugging
// purposes.
//
simple ContactPlanCreatingApp
{
    parameters:
        bool sending = default(false);                        // whether data is generated
        double startTime @unit(s) = default(0s);              // time at which generation is started
        double stopTime @unit(s) = default(-1s);              // time at which generation is stopped
        volatile double sendInterval @unit(s) = default(60s); // interval with which data is generated
        volatile int destinationNodeNo = default(1);          // destinated node of the generated packets
        volatile int payloadSize @unit(B) = default(100B);              // size of the generated packets
        int nodeNo;                                           // own node no
    gates:
        input lowerLayerIn;
        output lowerLayerOut;
}
