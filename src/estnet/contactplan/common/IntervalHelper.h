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

#ifndef UTILS_INTERVALHELPER_H_
#define UTILS_INTERVALHELPER_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <list>

#include "estnet/common/ESTNETDefs.h"

using namespace std;

namespace estnet {

struct ESTNET_API interval {
    int64_t start;
    int64_t end;
    interval(int64_t s, int64_t e) :
            start(s), end(e) {
    }
};

/*
 * Merges multiple time intervals into the smallest possible amount of intervals.
 */
list<interval> ESTNET_API mergeIntervals(vector<interval> &t);

/*
 * Helper for the merge interval method
 */
bool ESTNET_API intervalCompare(interval a, interval b);

}  // namespace estnet

#endif
