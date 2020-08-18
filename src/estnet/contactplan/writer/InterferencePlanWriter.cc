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

#include "InterferencePlanWriter.h"

#include <fstream> // ofstream
#include <algorithm> // stable_sort
#include <iomanip> // setiosflags
#include <cmath> // log10
#include <sstream> //write vector to string

namespace estnet {

void InterferencePlanWriter::write(
        const std::vector<interference_plan_entry_t*> &interferencePlan,
        const std::string &filename) {
    // first we'll sort all the interferences so the file is easier to read and
    // understand
    std::vector<interference_plan_entry_t*> interferences = interferencePlan;
    InterferencePlanEntryTimeFirstComparator cmp;
    std::stable_sort(interferences.begin(), interferences.end(), cmp);

    int64_t totalEndTime = 0;
    unsigned int maxNodeId = 0;
    unsigned int maxInterferingDigits = 0;

    for (const auto &interference : interferences) {
        if (interference->endTime > totalEndTime) {
            totalEndTime = interference->endTime;
        }
        if (interference->sourceNodeId > maxNodeId) {
            maxNodeId = interference->sourceNodeId;
        }
        if (interference->sinkNodeId > maxNodeId) {
            maxNodeId = interference->sinkNodeId;
        }
        unsigned int counter = 0;
        for (unsigned int id : interference->interferingNodeId) {
            counter += (int) log10(id) + 2;
        }
        counter--;
        if (counter > maxInterferingDigits) {
            maxInterferingDigits = counter;
        }
    }

    // figure out max number of digits for nice formatting
    unsigned int totalEndTimeNumDigits = std::max((int) log10(totalEndTime) + 1,
            10);
    unsigned int maxNodeIdNumDigits = std::max((int) log10(maxNodeId) + 1, 10);

    // log out contacts
    std::ofstream interferencePlanFile;
    interferencePlanFile.open(filename);
    interferencePlanFile << "# start(sec)   end(sec) interferer"
            << "     source       sink" << std::endl;
    for (const auto &interference : interferences) {
        if (interference->startTime == interference->endTime
                || interference->startTime > interference->endTime) {
            // we need at least one second of interference, otherwise we'll discard the
            // entry
            continue;
        }
        if (interference->enabled) {
            interferencePlanFile << "  ";
            interferencePlanFile << std::setw(totalEndTimeNumDigits)
                    << std::setiosflags(std::ios::right)
                    << interference->startTime << " ";
            interferencePlanFile << std::setw(totalEndTimeNumDigits)
                    << std::setiosflags(std::ios::right)
                    << interference->endTime << " ";
            interferencePlanFile << std::setw(maxInterferingDigits)
                    << std::setiosflags(std::ios::right)
                    << writeVector(interference->interferingNodeId) << " ";
            interferencePlanFile << std::setw(maxNodeIdNumDigits)
                    << std::setiosflags(std::ios::right)
                    << interference->sourceNodeId << " ";
            interferencePlanFile << std::setw(maxNodeIdNumDigits)
                    << std::setiosflags(std::ios::right)
                    << interference->sinkNodeId << std::endl;
        }
    }
    interferencePlanFile.close();
}

template<typename T>
std::string InterferencePlanWriter::writeVector(std::vector<T> vec) {
    sort(vec.begin(), vec.end());
    std::stringstream strStream;
    char str[3];
    sprintf(str, "%d", *vec.begin().base());
    strStream << str;
    vec.erase(vec.begin());
    for (auto element : vec) {
        sprintf(str, "%d", element);
        strStream << "," << str;
    }
    return strStream.str();
}

}  // namespace estnet
