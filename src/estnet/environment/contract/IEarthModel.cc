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

#include "IEarthModel.h"

namespace estnet {

void IEarthModel::convertECEFToLatLongHeight(const inet::Coord &ecef,
        inetu::deg &latitude, inetu::deg &longitude, inetu::m &altitude) const {
    this->convertECEFToLatLongHeight(inetu::m(ecef.x), inetu::m(ecef.y),
            inetu::m(ecef.z), latitude, longitude, altitude);
}

void IEarthModel::convertLatLongHeightToECEF(inetu::deg latitude,
        inetu::deg longitude, inetu::m altitude, inet::Coord &ecef) const {
    inetu::m x, y, z;
    this->convertLatLongHeightToECEF(latitude, longitude, altitude, x, y, z);

    ecef.x = x.get();
    ecef.y = y.get();
    ecef.z = z.get();
}

void IEarthModel::convertECIToLatLongHeight(cJulian time,
        const inet::Coord &eci, inetu::deg &latitude, inetu::deg &longitude,
        inetu::m &altitude) const {
    this->convertECEFToLatLongHeight(inetu::m(eci.x), inetu::m(eci.y),
            inetu::m(eci.z), latitude, longitude, altitude);
    // remove longitude advancement (Earth rotation) that ECEF coordinates
    // include
    longitude -= time.toGMST();
    if (longitude.get() < 0.0) {
        longitude = longitude + inetu::deg(360.0);
    }
}

void IEarthModel::convertLatLongHeightToECI(cJulian time, inetu::deg latitude,
        inetu::deg longitude, inetu::m altitude, inet::Coord &eci) const {
    // add longitude advancement (earth rotation) that ECI coordinates don't
    // include
    inetu::deg lmst = time.toLMST(longitude);
    inetu::m x, y, z;

    this->convertLatLongHeightToECEF(latitude, lmst, altitude, x, y, z);
    eci.x = x.get();
    eci.y = y.get();
    eci.z = z.get();
}

void IEarthModel::computeNadirCoordinateFrameFromLatLong(inetu::deg latitude,
        inetu::deg longitude, inet::Coord &east, inet::Coord &north,
        inet::Coord &up) const {
    // we're computing the same thing as OSG in EllipsoidModel::computeCoordinateFrame
    // so we can reverse it later, to get full control over the attitude

    // Compute up vector
    up.x = cos(longitude) * cos(latitude);
    up.y = sin(longitude) * cos(latitude);
    up.z = sin(latitude);

    // Compute east vector
    east.x = -sin(longitude);
    east.y = cos(longitude);
    east.z = 0;

    // Compute north vector = cross product up x east
    north = up % east;
}

void IEarthModel::computeNadirCoordinateFrameFromECI(cJulian time,
        const inet::Coord &eci, inet::Coord &east, inet::Coord &north,
        inet::Coord &up, reference_system rtnReferenceSystem) const {
    inetu::deg latitude, longitude;
    inetu::m altitude;

    this->convertECIToLatLongHeight(time, eci, latitude, longitude, altitude);

    switch (rtnReferenceSystem) {
    case ECI:
        // set back earth rotation to pseudo longitude, as conversion to
        // lat long does a transformation to ECEF before comuptation
        longitude += time.toGMST();
        if (longitude > inetu::deg(360)) {
            longitude -= inetu::deg(360);
        }
    case ECEF:
        this->computeNadirCoordinateFrameFromLatLong(latitude, longitude, east,
                north, up);
    }
}

void IEarthModel::computeNadirCoordinateFrameFromECEF(cJulian time,
        const inet::Coord &ecef, inet::Coord &east, inet::Coord &north,
        inet::Coord &up, reference_system rtnReferenceSystem) const {
    inetu::deg latitude, longitude;
    inetu::m altitude;

    this->convertECEFToLatLongHeight(ecef, latitude, longitude, altitude);

    switch (rtnReferenceSystem) {
    case ECI:
        // set back earth rotation to pseudo longitude, as conversion to
        // lat long does a transformation to ECEF before comuptation
        longitude -= time.toGMST();
    case ECEF:
        this->computeNadirCoordinateFrameFromLatLong(latitude, longitude, east,
                north, up);
    }
}

inetu::deg IEarthModel::calculateElevation(const inet::Coord &satPositionEci,
        const inetu::deg &latitude, const inetu::deg &longitude) const {
    inetu::m satAltitude;
    inetu::deg satLong, satLat;
    this->convertECIToLatLongHeight(
            GlobalJulianDate::getInstance().currentSimTime(), satPositionEci,
            satLat, satLong, satAltitude);

    // Elevation calculation: TODO: check if correct
    double angularDistance = acos(
            sin(satLat) * sin(latitude)
                    + cos(satLat) * cos(latitude)
                            * cos(
                                    inetu::deg(
                                            abs((satLong - longitude).get()))));
    double nadirAngle = atan(
            (EARTH_AVG_R * sin(angularDistance) / (satPositionEci.length()))
                    / (1
                            - (EARTH_AVG_R * cos(angularDistance)
                                    / (satPositionEci.length()))));
    return inetu::rad( M_PI / 2 - angularDistance - nadirAngle);
}
} // namespace estnet

