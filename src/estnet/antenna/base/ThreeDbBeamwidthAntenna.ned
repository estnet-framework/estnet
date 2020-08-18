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

package estnet.antenna.base;

//
// Base model for an antenna that can
// be modeled with an 3dB beamwidth
//
module ThreeDbBeamwidthAntenna extends AntennaBaseWithOrientation
{
    parameters:
        double maxGain @unit(dB);    // maximum gain of the antenna radiation pattern
        double minGain @unit(dB);    // minimum gain of the antenna
        double beamWidth @unit(deg); // 3dB beamwidth
        @class(ThreeDbBeamwidthAntenna);
}
