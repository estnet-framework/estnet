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

#ifndef ESTNET_NOISE_GROUNDSTATIONBACKGROUNDNOISE_H_
#define ESTNET_NOISE_GROUNDSTATIONBACKGROUNDNOISE_H_

#include <omnetpp.h>

#include "inet/physicallayer/contract/packetlevel/IBackgroundNoise.h"
#include "inet/physicallayer/contract/packetlevel/IAntenna.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"

#include "estnet/common/ESTNETDefs.h"
#include "estnet/mobility/terrestrial/StaticTerrestrialMobility.h"

#include "estnet/common/interpolation/multivariate-splines/datatable.h"
#include "estnet/common/interpolation/multivariate-splines/rbfspline.h"

namespace estnet {


/**
 *  Noise class to model the background noise of a ground station based on its current azimuth and elevation.
 *  The noise data needs to be provided by a csv file for different sections. The data is then interpolated
 *  based on the current orientation either using thin plane splines (https://github.com/lasote/multivariate-splines)
 *  or NDDNISD interpolation (https://arxiv.org/pdf/1910.00704.pdf)
 */
class ESTNET_API GroundstationBackgroundNoise: public cModule,
        public inet::physicallayer::IBackgroundNoise {
protected:
    virtual void initialize(int stage) override;

public:
    GroundstationBackgroundNoise() : samples(false, true) { // set DataTable to use irregular grid
    }
    ;

    virtual const inet::physicallayer::INoise* computeNoise(
            const inet::physicallayer::IListening *listening) const override;

private:
    // computes the grate circle distance between two points on a sphere
    double computeAngularDistance(double e0, double a0, double e1,
            double a1) const;

    // averages latitude/longitude (azimuth/elevation) by converting them to a vector on the unit sphere
    // then averages these vectors and converts them back to latitude and longitude
    std::pair<double, double> avarageLatLong(
            std::vector<std::pair<double, double> > &latLong) const;

    // element for storing the data
    struct noiseElement {
        double e_min, e_max; // min and max elevation [rad]
        double a_min, a_max; // min and max azimuth [rad]
        double power; // noise power [W]
    };

    // element to store noise grid data
    struct noiseGridElement {
        double e_min, e_max; // min and max elevation [rad]
        double a_min, a_max; // min and max azimuth [rad]
        double power0, power1, power2, power3; // noise power at the corners [W]
    };


    std::vector<noiseElement> data; // store the noise data elements
    std::vector<noiseGridElement> grid; // store grid for bilinear interpolation

    double halfAzimuthRes;
    int numOfInterpolationNeighbors;

    double minElevation;
    int interpolationMethod;
    bool regularGrid;

    MultivariateSplines::DataTable samples;
    MultivariateSplines::RBFSpline *rbfspline;
};

} //estnet

#endif /* ESTNET_NOISE_GROUNDSTATIONBACKGROUNDNOISE_H_ */
