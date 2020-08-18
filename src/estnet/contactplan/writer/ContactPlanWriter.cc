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

#include "ContactPlanWriter.h"

#include <fstream> // ofstream
#include <algorithm> // stable_sort
#include <iomanip> // setiosflags
#include <cmath> // log10

namespace estnet {

void ContactPlanWriter::write(
        const std::vector<contact_plan_entry_t*> &contactPlan,
        const std::string &filename, int64_t maxSimTime) {
    // first we'll sort all the contacts so the file is easier to read and
    // understand
    std::vector<contact_plan_entry_t*> contacts = contactPlan;
    ContactPlanEntryTimeFirstComparator cmp;
    std::stable_sort(contacts.begin(), contacts.end(), cmp);

    int64_t totalEndTime = 0;
    unsigned int maxNodeId = 0;
    unsigned int maxSatNodeId = 1;
    int64_t maxBitrate = 0;
    int64_t maxRange = 0;

    for (const auto &contact : contacts) {
        if (contact->endTime > totalEndTime) {
            totalEndTime = contact->endTime;
        }
        if (contact->sourceNodeId > maxNodeId) {
            maxNodeId = contact->sourceNodeId;
        }
        if (contact->sinkNodeId > maxNodeId) {
            maxNodeId = contact->sinkNodeId;
        }
        if (contact->sourceNodeId > maxSatNodeId
                && !contact->sourceIsGroundStation) {
            maxSatNodeId = contact->sourceNodeId;
        }
        if (contact->sinkNodeId > maxSatNodeId
                && !contact->sinkIsGroundStation) {
            maxSatNodeId = contact->sinkNodeId;
        }
        if (contact->bitrate > maxBitrate) {
            maxBitrate = contact->bitrate;
        }
        if (contact->range > maxRange) {
            maxRange = contact->range;
        }
    }

    // figure out max number of digits for nice formatting
    unsigned int totalEndTimeNumDigits = std::max((int) log10(totalEndTime) + 1,
            10);
    unsigned int maxNodeIdNumDigits = std::max((int) log10(maxNodeId) + 1, 6);
    unsigned int maxBitrateNumDigits = std::max((int) log10(maxBitrate) + 1, 9);
    unsigned int maxRangeNumDigits = log10(maxRange) + 1;

    // log out contacts
    std::ofstream contactPlanFile;
    contactPlanFile.open(filename);
    if (maxSatNodeId > 1) {
        contactPlanFile << "# Satellites: 1 - " << maxSatNodeId << std::endl;
    }
    if (maxSatNodeId < maxNodeId) {
        contactPlanFile << "# Ground Stations: " << maxSatNodeId + 1 << " - "
                << maxNodeId << std::endl;
    }
    contactPlanFile << "# sim-time-limit: " << maxSimTime << std::endl;
    contactPlanFile << "# start(sec)   end(sec) source "
            << "  sink rate(bps) range(lightseconds)" << std::endl;
    for (const auto &contact : contacts) {
        if (contact->startTime == contact->endTime
                || contact->startTime > contact->endTime) {
            // we need at least one second of contact, otherwise we'll discard the
            // contact
            continue;
        }
        if (contact->enabled) {
            contactPlanFile << "  ";
            contactPlanFile << std::setw(totalEndTimeNumDigits)
                    << std::setiosflags(std::ios::right) << contact->startTime
                    << " ";
            contactPlanFile << std::setw(totalEndTimeNumDigits)
                    << std::setiosflags(std::ios::right) << contact->endTime
                    << " ";
            contactPlanFile << std::setw(maxNodeIdNumDigits)
                    << std::setiosflags(std::ios::right)
                    << contact->sourceNodeId << " ";
            contactPlanFile << std::setw(maxNodeIdNumDigits)
                    << std::setiosflags(std::ios::right) << contact->sinkNodeId
                    << " ";
            contactPlanFile << std::setw(maxBitrateNumDigits)
                    << std::setiosflags(std::ios::right) << contact->bitrate
                    << " ";
            contactPlanFile << std::setw(maxRangeNumDigits)
                    << std::setiosflags(std::ios::right) << contact->range
                    << std::endl;
        }
    }
    contactPlanFile.close();
}

}  // namespace estnet
