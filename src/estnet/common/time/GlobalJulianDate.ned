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

package estnet.common.time;

//
// Module that provides time settings
// Maps the simulation time to a julian date referred to a given start time
//
simple GlobalJulianDate
{
    // Simulation date reference, if no TLE file with epoch is given
    string simulationStart = default(""); // allows only for ISO 8601 like: 2020-03-11T10:58:00.00+01:00Z

    // TLE file with three lines per satellite
    // Optional takes the epoch from the TLE in ithis file, otherwise it takes the simulation start value
    string tleFile = default("");
}
