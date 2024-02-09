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

#ifndef __ESTNET__STR_UTIL_H__
#define __ESTNET__STR_UTIL_H__

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using string = std::string;
using isis = std::istream_iterator<std::string>;
/** @brief splits a string and pushes the elements to a vector */
template <typename T>
void splitStringPushVector(const string &s, std::vector<T> &v) {
  std::istringstream iss(s);
  std::transform(isis(iss), isis(), std::back_inserter(v),
                 [](const string &s1) -> T {
                   T x;
                   std::stringstream ss(s1);
                   ss >> x;
                   return x;
                 });
}

} // namespace

/** @brief output helper for std::vector<size_t> */
std::ostream &operator<<(std::ostream &stream, const std::vector<size_t> &v);

#endif // __ESTNET__STR_UTIL_H__
