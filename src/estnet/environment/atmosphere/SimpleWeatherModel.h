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

#ifndef __ESTNET_SIMPLERAINMODEL_H_
#define __ESTNET_SIMPLERAINMODEL_H_

#include <inet/common/geometry/common/Coord.h>
#include <inet/common/Units.h>

#include "estnet/common/ESTNETDefs.h"
#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

enum rainDropDistribution {
    LP_L = 0, LP_H = 1, MP = 2, J_T = 3, J_D = 4, NUM_DISTRIBUTIONS = 5
};

struct RainAttenuationAB {
    double a[5];
    double b[5];
};

/**
 * Simple model for weather, hold constant values for the
 * whole Earth which are set via ned parameters
 */
class ESTNET_API SimpleWeatherModel: public omnetpp::cSimpleModule {
public:
    double calculateRainAttenuation(inetu::Hz f, const inet::Coord &positionEci1,
            const inet::Coord &positionEci2);

protected:

    /**
     * Read in parameters
     * Set up look-up table for a and b values
     */
    virtual void initialize();

private:
    /*
     * Set up look up table for a and b values
     */
    void setUpAandB(const char *filename);

    rainDropDistribution getRainDropDistribution(std::string name);

    double _rainIntensity;
    inetu::m _rainHeight;

    std::map<inetu::Hz, RainAttenuationAB> _abLookupTable;
    rainDropDistribution _rainDropDistribution;

    IEarthModel *_earthModel;   ///< earth model used for elevation calculations

};

} //namespace

#endif
