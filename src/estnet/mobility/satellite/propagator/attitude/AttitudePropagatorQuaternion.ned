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

package estnet.mobility.satellite.propagator.attitude;

import estnet.mobility.satellite.contract.IAttitudePropagator;

//
// Attitude propagator using quaternions
//
simple AttitudePropagatorQuaternion like IAttitudePropagator
{
    parameters:
        double dT @unit(s) = default(0.01s);
        // Minimal Propagation Timestep in SimTime seconds
        
        double minConstT @unit(s) = default(1s);
        // Minimal duration after which an propagation update is needed for this specific propagator
        // in SimTime seconds, e.g. the time duration for which the system states are assumed constant
        
        
        double dSAccMin = default(0.001);
		// Minimal accuracy for each component of the state vector above which a new propagation is triggered
}
