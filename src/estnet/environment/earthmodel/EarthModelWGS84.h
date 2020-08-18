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

#ifndef __EARTH_MODEL_EARTH_MODEL_WGS84_H__
#define __EARTH_MODEL_EARTH_MODEL_WGS84_H__

#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

/**
 * Models the Earth according to WGS84
 */
class ESTNET_API EarthModelWGS84: public IEarthModel {
public:
    /** @brief Constructs the model with the default earth radii */
    EarthModelWGS84() :
            EarthModelWGS84(inetu::m(WGS_84_RADIUS_POLAR),
                    inetu::m(WGS_84_RADIUS_EQUATOR)) {
    }

    /** @brief Constructs the model with the given earth radii */
    EarthModelWGS84(inetu::m radiusPolar, inetu::m radiusEquator) :
            _radiusPolar(radiusPolar), _radiusEquator(radiusEquator), _eccentricitySquared(
                    2
                            * (((radiusEquator - radiusPolar) / radiusEquator)
                                    - ((radiusEquator - radiusPolar)
                                            / radiusEquator)
                                            * ((radiusEquator - radiusPolar)
                                                    / radiusEquator)).get()) {
    }

    /**
     * @brief Converts the given ECEF cartesian coordinates to
     * latitude, longitude and altitude according to the WGS84 model
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
        // http://www.colorado.edu/geography/gcraft/notes/datum/gif/xyzllh.gif
        inetu::m p = sqrt(x * x + y * y);
        inetu::rad theta = inetu::rad(
                atan2((z * _radiusEquator).get(), (p * _radiusPolar).get()));
        auto eDashSquared = (_radiusEquator * _radiusEquator
                - _radiusPolar * _radiusPolar) / (_radiusPolar * _radiusPolar);

        double sin_theta = sin(theta);
        double cos_theta = cos(theta);

        latitude = inetu::rad(
                atan(
                        ((z
                                + eDashSquared * _radiusPolar * sin_theta
                                        * sin_theta * sin_theta)
                                / (p
                                        - _eccentricitySquared * _radiusEquator
                                                * cos_theta * cos_theta
                                                * cos_theta)).get()));
        longitude = inetu::rad(atan2(y.get(), x.get()));

        double e_sin_latitude_sq = _eccentricitySquared * sin(latitude)
                * sin(latitude);
        inetu::m N = _radiusEquator / sqrt(1.0 - e_sin_latitude_sq);

        altitude = p / cos(latitude) - N;
    }

    /**
     * @brief Converts the latitude, longitude and altitude
     * to ECEF cartesian coordinates according to the WGS84 model
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

        // for details on maths see
        // http://www.colorado.edu/geography/gcraft/notes/datum/gif/llhxyz.gif
        double sin_latitude = sin(latitude);
        double cos_latitude = cos(latitude);
        inetu::m N = _radiusEquator
                / sqrt(
                        1.0
                                - _eccentricitySquared * sin_latitude
                                        * sin_latitude);
        x = (N + altitude) * cos_latitude * cos(longitude);
        y = (N + altitude) * cos_latitude * sin(longitude);
        z = (N * (1 - _eccentricitySquared) + altitude) * sin_latitude;
    }

private:
    const inetu::m _radiusPolar;   // measured in meters
    const inetu::m _radiusEquator; // measured in meters
    const double _eccentricitySquared;
};

}  // namespace estnet

#endif
