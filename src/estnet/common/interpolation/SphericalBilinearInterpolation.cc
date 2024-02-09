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

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <estnet/common/StrUtil.h>

#include "SphericalBilinearInterpolation.h"

SphericalBilinearInterpolation::SphericalBilinearInterpolation() {
}
;

SphericalBilinearInterpolation::SphericalBilinearInterpolation(
        const std::string &path) {
    load(path);
}

void SphericalBilinearInterpolation::load(const std::string &path) {
    min_lat = std::nan("");
    max_lat = std::nan("");
    lon_period = std::nan("");
    lon_offset = std::nan("");
    N_lat = 0;
    N_lon = 0;
    delta_lat = std::nan("");
    delta_lon = std::nan("");
    values.clear();
    std::ifstream f(path);
    if (!f)
        throw std::runtime_error("Noise map file not found.");
    std::string line;
    std::getline(f, line);
    if (line
            != "# min_lat max_lat lon_period lon_offset N_lat N_lon multiplier")
        throw std::runtime_error("bad line 1 in noise map file");
    std::getline(f, line);
    double multiplier = 1.0;
    {
        // read parameters:
        std::istringstream iss(line);
        iss >> min_lat;
        iss >> max_lat;
        if (!(min_lat < max_lat))
            throw std::runtime_error(
                    "requirement `min_lat < max_lat` not fulfilled");
        iss >> lon_period;
        if (!(lon_period > 0))
            throw std::runtime_error(
                    "requirement `lon_period > 0` not fulfilled");
        iss >> lon_offset;
        iss >> N_lat;
        if (!(N_lat >= 2))
            throw std::runtime_error("requirement `N_lat >= 2` not fulfilled");
        iss >> N_lon;
        if (!(N_lon >= 1))
            throw std::runtime_error("requirement `N_lon >= 1` not fulfilled");
        iss >> multiplier;
    }
    std::getline(f, line);
    if (line != "# values")
        throw std::runtime_error("bad line 1");
    // allocate memory:
    values.reserve(N_lat * N_lon);
    // read values:
    while (std::getline(f, line))
        splitStringPushVector(line, values);
    if (values.size() != N_lat * N_lon)
        throw std::runtime_error(
                "number of values read does not equal N_lat * N_lon");
    // rescale with multiplier:
    for (auto &x : values)
        x *= multiplier;
    // compute deltas:
    delta_lat = (max_lat - min_lat) / (N_lat - 1);
    delta_lon = lon_period / N_lon;
}

double SphericalBilinearInterpolation::get_value(const size_t n,
        const size_t m) const {
    if (!((0 <= n) && (n < N_lat) && (0 <= m) && (m < N_lon)))
        throw std::runtime_error("out of range");
    return values[N_lon * n + m];
}

double SphericalBilinearInterpolation::get(const double lat,
        const double lon) const {
    // latitude
    const double y = (lat - min_lat) / delta_lat;
    size_t n, np1;
    double fy;
    if (y < 0) {
        n = 0;
        np1 = 0;
        fy = 0.0;
    } else if (y >= N_lat - 1) {
        n = N_lat - 1;
        np1 = N_lat - 1;
        fy = 0.0;
    } else {
        n = y;
        np1 = n + 1;
        fy = y - n;
    }
    // longitude
    double x = (lon - lon_offset) / delta_lon;
    // true modulus calculation:
    x -= N_lon * std::floor(x / N_lon);
    if (x >= N_lon)
        x = 0;
    size_t m, mp1;
    m = std::floor(x);
    mp1 = m + 1;
    if (mp1 == N_lon)
        mp1 = 0;
    double fx = x - m;
#ifndef NDEBUG
    if (!((0 <= fx) && (fx < 1.0)))
        throw std::runtime_error("out of range: fx");
    if (!((0 <= fy) && (fy < 1.0)))
        throw std::runtime_error("out of range: fy");
#endif
    double result = (get_value(n, m) * (1 - fx) + get_value(n, mp1) * fx)
            * (1 - fy);
    if (fy > 0)
        result += (get_value(np1, m) * (1 - fx) + get_value(np1, mp1) * fx)
                * fy;
    return result;
}
