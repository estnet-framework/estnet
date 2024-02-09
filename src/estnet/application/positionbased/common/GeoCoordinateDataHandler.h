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

#ifndef __APPS__GEO_COORDINATE_DATA_HANDLER_H__
#define __APPS__GEO_COORDINATE_DATA_HANDLER_H__

#include <string>

#include "estnet/application/contract/IPositionData.h"

namespace estnet {

/**
 *  A single point described by latitude and longitude is the data source.
 */

class ESTNET_API GeoCoordinateDataHandler: public IPositionData {
public:
    GeoCoordinateDataHandler(std::string coord);
    virtual ~GeoCoordinateDataHandler();
    /** @brief gives multiplier for one specific longitude and latitude*/
    virtual void getDataForPoint(double latitude, double longitude,
            double &multiplier) override;
    /** @ brief gives the sum of all multipliers inside the beam*/
    virtual void getDataForCone(double latitude, double longitude,
            double altitude, double beamWidth, double &multiplier) override;
private:
    double latitude, longitude;
};

}  // namespace estnet

#endif
