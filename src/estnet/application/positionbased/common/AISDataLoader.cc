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

#include "AISDataLoader.h"

#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>

#include <inet/common/INETMath.h>

#include "estnet/global_config.h"

namespace estnet {

AISDataLoader::AISDataLoader(std::string path) {

    //reads data file
    file.open(path);
    //checks if file is open
    if (file.is_open()) {
        //Iterates over all data
        for (unsigned int i = 0; i < 64800; i++) {
            std::string line;
            std::getline(file, line);

            if (line.find("ID") != 0) {
                // search for the longitude value
                int j = line.find('\t') + 1;
                int k = line.find('\t', j);
                int longitude = std::stoi(line.substr(j, k - j));

                // search for the latitude value
                j = line.find('\t', k) + 1;
                j = line.find('\t', j) + 1;
                k = line.find('\t', j);
                int latitude = std::stoi(line.substr(j, k - j));

                // search for the multiplier value
                j = line.find('\t', k) + 1;
                j = line.find('\t', j) + 1;
                k = line.find('\t', j);
                std::string s = line.substr(j, k - j);
                std::replace(s.begin(), s.end(), ',', '.');
                // saves multiplier in data array and add length to compensate negative values
                data[latitude + 90][longitude + 180] = std::stof(s);
            }
        }

        file.close();
    } else {
        std::ostringstream oss;
        oss << "AIS data loader file: " << path << " not found.";
        const char *errorMsg = oss.str().c_str();
        throw omnetpp::cRuntimeError(errorMsg);
    }
}

AISDataLoader::~AISDataLoader() {
    delete &file;
}

void AISDataLoader::getDataForPoint(double latitude, double longitude,
        double &multiplier) {
    // gets multiplier for one point
    int lon = std::floor(longitude) + 180;
    int lat = std::floor(latitude) + 90;
    multiplier = data[lat][lon];
}

void AISDataLoader::getDataForCone(double latitude, double longitude,
        double altitude, double beamWidth, double &multiplier) {
    multiplier = 0;
    double alt = std::round(altitude);
    // calculates the radius of a circle around the center point in degree,
    // which can be directly added to latitude/longitude
    double alpha = beamWidth / 2.0;
    double r = 180.0
            - (180.0
                    - inet::math::rad2deg(
                            std::asin(
                                    alt * std::sin(inet::math::deg2rad(alpha))
                                            / EARTH_AVG_R))) - alpha;
    // iterates over a 2r by 2r square and finds all points inside a circle with radius r
    // and center point lat/lon
    for (int i = std::floor(latitude - r + 90);
            i <= std::floor(latitude + r + 90); i++) {
        for (int j = std::floor(longitude - r + 180);
                j <= std::floor(longitude + r + 180); j++) {
            if (std::pow(i - (latitude + 90), 2)
                    + std::pow(j - (longitude + 180), 2) <= std::pow(r, 2)) {
                //adjusts the coordinates if they are over the array boundaries
                int ii, jj;
                if (i < 0) {
                    ii = std::abs(i);
                } else if (i > 180) {
                    ii = 180 - (i % 180);
                } else {
                    ii = i;
                }
                if (j < 0) {
                    jj = 360 + j;
                } else {
                    jj = j % 360;
                }
                multiplier += data[ii][jj];
            }
        }
    }
}

}  // namespace estnet
