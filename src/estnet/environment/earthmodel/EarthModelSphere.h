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

#ifndef __EARTH_MODEL_EARTH_MODEL_SPHERE_H__
#define __EARTH_MODEL_EARTH_MODEL_SPHERE_H__

#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

/**
 * Models the Earth as a Sphere
 */
class ESTNET_API EarthModelSphere: public IEarthModel {
public:
    /** @brief Constructs the model with the default earth radius */
    EarthModelSphere() :
            EarthModelSphere(inetu::m(EARTH_AVG_R)) {
    }
    ;

    /** @brief Constructs the model with the given radius */
    explicit EarthModelSphere(inetu::m r) :
            R(r) {
    }
    ;

    /**
     * @brief Converts the given ECEF cartesian coordinates to
     * latitude, longitude and altitude according to the sphere model
     * @param x coordinate in meters, for which the geocentric coordinates
     *  are determined
     * @param y coordinate in meters, for which the geocentric coordinates
     *  are determined
     * @param z coordinate in meters, for which the geocentric coordinates
     *  are determined
     * @param latitude in degrees, reference as return value
     * @param longitude in degrees, reference as return value
     * @param altitude in meters, reference as return value
     */
    virtual void convertECEFToLatLongHeight(inetu::m x, inetu::m y, inetu::m z,
            inetu::deg &latitude, inetu::deg &longitude,
            inetu::m &altitude) const {
        inetu::m equatorial_length = sqrt(x * x + y * y);
        latitude = inetu::rad(atan((z / equatorial_length).get()));
        longitude = inetu::rad(atan2(y.get(), x.get()));
        altitude = equatorial_length - R;
    }
    ;

    /**
     * @brief Converts the latitude, longitude and altitude
     * to ECEF cartesian coordinates according to the sphere model
     * @param latitude in degrees, for which ECEF cartesian coordinates
     *  are determined
     * @param longitude in degrees, for which ECEF cartesian coordinates
     *  are determined
     * @param altitude in meters, for which ECEF cartesian coordinates
     *  are determined
     * @param x coordinate in meters, reference as return value
     * @param y coordinate in meters, reference as return value
     * @param z coordinate in meters, , reference as return value
     */
    virtual void convertLatLongHeightToECEF(inetu::deg latitude,
            inetu::deg longitude, inetu::m altitude, inetu::m &x, inetu::m &y,
            inetu::m &z) const {
        x = (R + altitude) * cos(latitude) * cos(longitude);
        y = (R + altitude) * cos(latitude) * sin(longitude);
        z = (R + altitude) * sin(latitude);
    }
    ;

private:
    const inetu::m R; // sphere radius
};

}  // namespace estnet

#endif
