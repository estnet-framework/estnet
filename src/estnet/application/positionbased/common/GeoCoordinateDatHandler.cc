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

#include "GeoCoordinateDataHandler.h"

#include <regex>
#include <algorithm>
#include <cmath>

#include <inet/common/INETMath.h>

#include "estnet/global_config.h"

namespace estnet {

GeoCoordinateDataHandler::GeoCoordinateDataHandler(std::string coord) {
    std::smatch m;
    std::regex r_lat("^(\\-?\\d+(\\.\\d+)?)(?=,\\s)");
    std::regex r_lon("(\\-?\\d+(\\.\\d+)?)$");
    std::regex_search(coord, m, r_lat);
    this->latitude = stod(m[1]);
    std::regex_search(coord, m, r_lon);
    this->longitude = stod(m[1]);
}

GeoCoordinateDataHandler::~GeoCoordinateDataHandler() {
}

void GeoCoordinateDataHandler::getDataForPoint(double latitude,
        double longitude, double &multiplier) {
    if (latitude == this->latitude && longitude == this->longitude) {
        multiplier = 1;
    }
    multiplier = 0;
}

void GeoCoordinateDataHandler::getDataForCone(double latitude, double longitude,
        double altitude, double beamWidth, double &multiplier) {
    multiplier = 0;
    // calculates the radius of a circle around the center point in degree,
    // which can be directly added to latitude/longitude
    double alpha = beamWidth / 2.0;
    double r = 90.0 - alpha;
    if (std::pow(this->latitude - latitude, 2)
            + std::pow(this->longitude - longitude, 2) <= std::pow(r, 2)) {
        multiplier = 1;
    }
}

}  // namespace estnet
