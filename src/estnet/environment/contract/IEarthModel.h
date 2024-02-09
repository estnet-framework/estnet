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

#ifndef __EARTH_MODEL_IEARTH_MODEL_H__
#define __EARTH_MODEL_IEARTH_MODEL_H__

#include <inet/common/geometry/common/Coord.h>
#include <inet/common/Units.h>

#include "estnet/global_config.h"
#include "estnet/common/time/GlobalJulianDate.h"

namespace estnet {

enum reference_system {
    ECI = 0, ECEF = 1
};

/**
 * Interface for earth models
 */
class ESTNET_API IEarthModel {
public:
    virtual ~IEarthModel() = default;

    /**
     * @brief Converts ECEF cartesian coordinates to latitude, longitude, altitude
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
    virtual void convertECEFToLatLongHeight(inetu::m X, inetu::m Y, inetu::m Z,
            inetu::deg &latitude, inetu::deg &longitude,
            inetu::m &altitude) const = 0;

    /**
     * @brief Converts latitude, longitude, altitude to ECEF cartesian coordinates
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
            inetu::deg longitude, inetu::m altitude, inetu::m &X, inetu::m &Y,
            inetu::m &Z) const = 0;

    /**
     * @brief Converts ECEF cartesian coordinates to latitude, longitude, altitude
     * @param ecef: coordinates in meters, for which the geocentric coordinates
     *  are determined
     * @param latitude in degrees, reference as return value
     * @param longitude in degrees, reference as return value
     * @param altitude in meters, reference as return value
     */
    virtual void convertECEFToLatLongHeight(const inet::Coord &ecef,
            inetu::deg &latitude, inetu::deg &longitude,
            inetu::m &altitude) const;

    /**
     * @brief Converts latitude, longitude, altitude to ECEF cartesian coordinates
     * @param latitude in degrees
     * @param longitude in degrees
     * @param altitude in meters
     * @param ecef: coordinates in meters, that are determined by the
     *  geocentric coordinates passed, reference as return value
     */
    virtual void convertLatLongHeightToECEF(inetu::deg latitude,
            inetu::deg longitude, inetu::m altitude, inet::Coord &ecef) const;

    /**
     * @brief converts ECI coordinates to latitude, longitude and altitude
     * @param time: julian date that sets the coordinates in the correct context
     *  considering earth rotation
     * @param eci: coordinates in meters, for which the geocentric coordinates
     *  are determined
     * @param latitude in degrees, reference as return value
     * @param longitude in degrees, reference as return value
     * @param altitude in meters, reference as return value
     */
    virtual void convertECIToLatLongHeight(cJulian time, const inet::Coord &eci,
            inetu::deg &latitude, inetu::deg &longitude,
            inetu::m &altitude) const;

    /**
     * @brief converts latitude, longitude and altitude to ECI coordinates
     * @param latitude in degrees
     * @param longitude in degrees
     * @param altitude in meters
     * @param eci: coordinates in meters, that are determined by the
     *  geocentric coordinates passed, reference as return value
     */
    virtual void convertLatLongHeightToECI(cJulian time, inetu::deg latitude,
            inetu::deg longitude, inetu::m altitude, inet::Coord &eci) const;

    /*
     * @brief calculates an ENU coordinate frame based on the given
     * ECEF coordinates
     * @param ECEF coordinates for which the ENU frame is calculated
     * @param east vector in defined frame, reference as return value
     * @param north vector in defined frame, reference as return value
     * @param up vector in defined frame, reference as return value
     * @param rtnReferenceSystem defines in which reference frame
     * the ENU vectors are described
     */
    virtual void computeNadirCoordinateFrameFromECEF(cJulian time,
            const inet::Coord &ecef, inet::Coord &east, inet::Coord &north,
            inet::Coord &up, reference_system rtnReferenceSystem = ECI) const;

    /*
     * @brief calculates an ENU coordinate frame based on the given
     * ECI coordinates
     * @param ECI coordinates for which the ENU frame is calculated
     * @param east vector in defined frame, reference as return value
     * @param north vector in defined frame, reference as return value
     * @param up vector in defined frame, reference as return value
     * @param rtnReferenceSystem defines in which reference frame
     * the ENU vectors are described
     */
    virtual void computeNadirCoordinateFrameFromECI(cJulian time,
            const inet::Coord &eci, inet::Coord &east, inet::Coord &north,
            inet::Coord &up, reference_system rtnReferenceSystem = ECI) const;

    /** @brief calculate the elevation of a point relative to a point
     *  given in geocentric coordinaties
     *  @param satellite's position as Eci-coordinate to which the elevation is
     *  determined
     *  @param latitude of geocentric point
     *  @param longitude of geocentric point
     *  @param altitude of geocentric point
     *  @return elevation angle of the given point relative to the
     *      geocentric coordinaties
     */
    virtual inetu::deg calculateElevation(const inet::Coord &satPositionEci,
            const inetu::deg &latitude, const inetu::deg &longitude,
            const inetu::m &altitude) const;

    /** @brief calculate the elevation of a point relative to a point
     *  given both in ECI coordinates. THe function automatically choose the
     *  point with lower elevation as ground point
     *  @param first position as Eci-coordinate to which the elevation is
     *  determined
     *  @param second position as Eci-coordinate to which the elevation is
     *  determined
     *  @return elevation angle between the given points
     */
    inetu::deg calculateElevation(const inet::Coord &positionEci1,
            const inet::Coord &positionEci2) const;

    /** @brief calculate the elevation of a point relative to a point
     *  given both in ECI coordinates. THe function automatically choose the
     *  point with lower elevation as ground point
     *  @param latitude1 of first geocentric point
     *  @param longitude1 of first geocentric point
     *  @param altitude1 of first geocentric point
     *  @param latitude2 of second geocentric point
     *  @param longitude2 of second geocentric point
     *  @param altitude2 of second geocentric point
     *  @return elevation angle between the given geocentric points
     */
    inetu::deg calculateElevation(const inetu::deg &lat1,
            const inetu::deg &long1, const inetu::m &alt1,
            const inetu::deg &lat2, const inetu::deg &long2,
            const inetu::m &alt2) const;

private:

    /*
     * @brief calculates an ENU coordinate frame based on the given
     * latitude and longitude.
     * This has to be the same coordinate frame as used by OSG and osgEarth.
     * @param latitude in degrees
     * @param longitude in degrees
     * @param east vector in ECEF frame, reference as return value
     * @param north vector in ECEF frame, reference as return value
     * @param up vector in ECEF frame, reference as return value
     */
    virtual void computeNadirCoordinateFrameFromLatLong(inetu::deg latitude,
            inetu::deg longitude, inet::Coord &east, inet::Coord &north,
            inet::Coord &up) const;
};

}  // namespace estnet

#endif
