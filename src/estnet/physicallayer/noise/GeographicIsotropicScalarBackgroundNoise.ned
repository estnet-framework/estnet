//
// Copyright (C) 2020 Computer Science VII: Robotics and Telematics -
// Julius-Maximilians-Universität Würzburg
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

package estnet.physicallayer.noise;

import inet.physicallayer.contract.packetlevel.IBackgroundNoise;

//
// This background noise model describes noise that does not change over space,
// time and frequency. It produces scalar noise signals that can be further used
// in scalar computations.
//
module GeographicIsotropicScalarBackgroundNoise like IBackgroundNoise
{
    parameters:
        double power @unit(dBm); // default noise power (for anything but SatMobility)
        string noiseMapPath;     // path to the noise map ascii file fed into SphericalBilinearInterpolation
        @class(GeographicIsotropicScalarBackgroundNoise);
        @display("i=block/mac");
}

