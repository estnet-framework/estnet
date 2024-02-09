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

package estnet.physicallayer.noise;

import inet.physicallayer.contract.packetlevel.IBackgroundNoise;

//
// This module is used to select two distinct BackgroundNoise models for
// satellites and groundstations.
//
module BackgroundNoiseSelector like IBackgroundNoise
{
    parameters:
        @class(BackgroundNoiseSelector);
    submodules:
        // BackgroundNoise model for the satellite
        satelliteBackgroundNoise: <default("IsotropicScalarBackgroundNoise")>  like IBackgroundNoise  if typename != ""  {}
        // BackgroundNoise model for the groundstation
        groundStationBackgroundNoise: <default("GroundstationBackgroundNoise")>  like IBackgroundNoise  if typename != ""  {}
}