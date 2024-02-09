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


#ifndef __SPHERICAL_BILINEAR_INTERPOLATION_H__
#define __SPHERICAL_BILINEAR_INTERPOLATION_H__

#include <string>
#include <vector>


/**

 Implements a generic bilinear (two-dimensional linear) interpolation for
 a given rectangular grid of function values. One coordinate (the longitude) is
 assumed periodic, the other (latitude) not.

 As lat and lon suggest, this can be used to interpolate a sampled function
 over geographic coordinates.

 Its primary intention is to be used for implementation of a geo-dependent
 noise model, e.g., based on this kind of data:

 http://www7.informatik.uni-wuerzburg.de/forschung/space-exploration/projects/uwe-3/uwe-3-news/single-news/news/uwe-3-in-orbit-interference-analysis/

*/
class SphericalBilinearInterpolation {
public:
    SphericalBilinearInterpolation();
    SphericalBilinearInterpolation(const std::string &path);
    void load(const std::string &path);
    double get(const double lat, const double lon) const;
    double get_value(const size_t n, const size_t m) const;

private:
    double min_lat;
    double max_lat;
    double lon_period;
    double lon_offset;
    size_t N_lat;
    size_t N_lon;
    double delta_lat;
    double delta_lon;
    std::vector<double> values;
};

#endif // __SPHERICAL_BILINEAR_INTERPOLATION_H__
