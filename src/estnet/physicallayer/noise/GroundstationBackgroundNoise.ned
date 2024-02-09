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


// Noise to model the background noise of a ground station based on its current azimuth and elevation.
// The noise data needs to be provided by a csv file for different sections. The data is then interpolated
// based on the current orientation using different interpolation methods.
// The options are:
// Bilinear Interpolation	(https://en.wikipedia.org/wiki/Bilinear_interpolation)
// 		requires a regular aranged grid in the data
// Thin plate splines		(https://github.com/lasote/multivariate-splines)
// NDDNISD interpolation	(https://arxiv.org/pdf/1910.00704.pdf)

module GroundstationBackgroundNoise like IBackgroundNoise
{
    parameters:
        double minElevation @unit(rad) = default(0.0rad); // If the elevation is blow this value the noise is set to infinity
        string noiseMapPath; // path to the noise data of the groundstation

        // select the interpolation method to be used:
        // 0: Bilinear Interpolation	(https://en.wikipedia.org/wiki/Bilinear_interpolation)
        // 1: Thin plate splines		(https://github.com/lasote/multivariate-splines)
        // 2: NDDNISD interpolation		(https://arxiv.org/pdf/1910.00704.pdf)
        int interpolationMethod = default(0);

        bool regularGrid = default(true); // select wether the noise data is arranged in a regular grid (required for bilinear interpolation)

        // number of closest neighbor the interpolation should consider (ONLY USED IF: interpolationMethod = 2)
        // (lower value - less smooth output; higher value - more smoothed data, but local peaks might be lost)
        int numOfInterpolationNeighbors = default(7);
        @class(GroundstationBackgroundNoise);
        @display("i=block/mac");
}
