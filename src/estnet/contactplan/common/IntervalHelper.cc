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

#include "IntervalHelper.h"

namespace estnet {

bool intervalCompare(interval a, interval b) {
    if (a.start < b.start) {
        return true;
    }
    return false;
}

list<interval> mergeIntervals(vector<interval> &t) {

    list<interval> stack;

    if (t.empty())
        return stack;

    sort(t.begin(), t.end(), intervalCompare);
    stack.push_back(t[0]);

    for (size_t i = 1; i < t.size(); i++) {
        interval top = stack.back();
        if (top.end < t[i].start) {
            stack.push_back(t[i]);
        } else if (top.end < t[i].end) {
            top.end = t[i].end;
            stack.pop_back();
            stack.push_back(top);
        }
    }
    return stack;
}

}  // namespace estnet
