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

import estnet.node.groundstation.GroundLabel;

// 
// Represents a jamming station/signal located on earth, which can cause link failures 
//
simple JammingStation extends GroundLabel
{
    double jammingProbability = default(0.8); // propability of link interference
    double minElevation @unit(deg) = default(0deg); // minimum neede elevation need befor jaming can occure
    double bandwidth @unit(Hz); // jaming bandwidth
    double centerFrequency @unit(Hz); // jaming center frequency
    modelScale = default(100000);
    @class(JammingStation);
}
