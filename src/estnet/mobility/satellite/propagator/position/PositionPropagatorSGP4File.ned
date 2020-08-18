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

package estnet.mobility.satellite.propagator.position;

import estnet.mobility.satellite.contract.IPositionPropagator;

//
// Propagates satellite position using the SGP4 model
// TLEs are read from a TLE file
//
simple PositionPropagatorSGP4File like IPositionPropagator
{
    parameters:
        // TLE file with three lines per satellite
        // The exact TLE is figured out based on the satellite nodeId
        // which is retrieved from the parentModules parameter "nodeNo".
        string tleFile;
        // Minimal accuracy for each component of the state vector above which a new propagation is triggered
        double dSAccMin = default(0.1);
}
