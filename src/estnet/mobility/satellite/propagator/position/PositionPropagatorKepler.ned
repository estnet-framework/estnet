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

// represents satellite propagation element.
// initializing and updating the OSG 3D visualization
simple PositionPropagatorKepler like IPositionPropagator
{
    parameters:

        // kelperian orbit parameters. default: molniya-orbit
        
        double a @unit(km) = default(26554.0km);				// semimajor axis [km]
        double e = default(0.72);								// excentricity
        double i @unit(deg) = default(63.4deg);					// inclination [deg]
        double raan @unit(deg) = default(0.0deg);				// right ascension of the ascending node [deg]
        double aop @unit(deg) = default(0.0deg);				// argument of perigee [deg]
        double v  @unit(deg) = default(0.0deg);					// initial true anomaly [deg]

        double dSAccMin @unit(m) = default(0.1m);
		// Minimal accuracy for each component of the state vector above which a new propagation is triggered
}
