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

import estnet.siminterface.pubsub.Publisher;


//
// represents a module on satellites that models failures on the connectivity
// it can simulate node failures (e.g. latch-ups) and it handles the desicion on whether a jamming station is jamming the satellite
//
simple NodeFailureModel like Publisher
{
    parameters:
        bool enable = default(true);	// if node failure should be enabled or not
        int faultSeed = default(0);		// random seed for failures
        int checkIntervall = default(60);	// intervall at which should be check if a new failure occures
        double MTTF @unit(s);	// mean time to failure; mean value at which a failure can occure
        double MTTR @unit(s);	// mean time to rapair; mean value at which a failure gets repaired

        @signal[PubSubSignal];
        @signal[nodeFailed](type=bool);
        @statistic[nodeFailed](source=nodeFailed; record=vector);
}
