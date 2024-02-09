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

package estnet.node.tracking.swipe;
import estnet.node.tracking.contract.ISwipeTracking;

//
// Swipe algorithm that uses a exponentially increasing/decreasing
// angular velocity
//
simple ExponentialSwipeTracking like ISwipeTracking
{
    double updateInterval @unit(s) = default(0.1s); // minimum update intervall of tracking orientation
    int startNodeId;                // first satellite id in formation
    int stopNodeId;                 // last satellite id in formation
    bool enable = default(true);    // enable tracking by default

    double exponent = default(2);      // exponent used in tracking function
                                    // ( the higher, the longer outer satellites are tracked)


    @signal[relTarget](type=double);
    @statistic[relTarget](title="RelativeTargetWithinFormation"; record=vector,histogram);
}
