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

#ifndef APPS_IPOSITIONDATA_H_
#define APPS_IPOSITIONDATA_H_

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 *  Interface to provide data for the PositionBasedApp. The data is either accessed with a single
 *  point or a cone.
 */
class ESTNET_API IPositionData {
public:
    /** @brief gets the multiplier for on point*/
    virtual void getDataForPoint(double latitude, double longitude,
            double &multiplier) = 0;
    /** @brief gets the sum of all multipliers within a cone*/
    virtual void getDataForCone(double latitude, double longitude,
            double altitude, double beamWidth, double &multiplier) = 0;
};

}  // namespace estnet

#endif /* APPS_IPOSITIONDATA_H_ */
