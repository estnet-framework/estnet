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

package estnet.noise;

import inet.physicallayer.contract.packetlevel.IBackgroundNoise;

//
// Noise model that acounts for the dirrection the satellite is pointing
// and how much of its viewing cone is looking at the earth or space.
//
module DirectionalScalarBackgroundNoise like IBackgroundNoise
{
    parameters:
        double receiverNoiseTemp @unit(K) = default(614K);	// noise of the reciever
        double t_Earth @unit(K) = default(290K); 			// noise temprature of the earth
        double t_Space @unit(K) = default(2.7K); 			// noise temprature of the space
        @class(DirectionalScalarBackgroundNoise);
        @display("i=block/mac");
}

