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



package estnet.antenna.contactplan;

import inet.physicallayer.base.packetlevel.AntennaBase;

//
// Isotropic Antenna with a gain parameter, used for contact plan creation:
// It makes use of the ini file property so that when mentioned
// before the normally used antenna, it transfers the belonging 
// gain parameter (refering to the gain for optimal pointing) into
// a omnidirectional like antenna that can be used for cp creation.
//
module OmnidirectionalAntennaCpr extends AntennaBase
{
    parameters:
        double maxGain @unit(dB) = default(0dB);	// maximum gain of the antenna radiation pattern
        double gain @unit(dB) = default(0dB);  		// gain of the antenna with gain name
        double baseGain @unit(dB) = default(0dB); 	// gain of the antenna with gain name
        double beamWidth @unit(deg) = 360deg; 		// 3dB beamwidth
        @class(OmnidirectionalAntennaCpr);
}

