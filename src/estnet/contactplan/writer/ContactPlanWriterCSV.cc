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

#include "ContactPlanWriterCSV.h"

#include <fstream> // ofstream
#include <algorithm> // stable_sort
#include <iomanip> // setiosflags
#include <cmath> // log10

namespace estnet {

void ContactPlanWriterCSV::write(
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
    }

    // log out contacts
    std::ofstream contactPlanFile;
    contactPlanFile.open(filename);

    // print out max-sim-time
    contactPlanFile << maxSimTime << ";";
    // print out num-sat
    contactPlanFile << maxSatNodeId << ";";
    // print out num-gs, this is node number minus num sat
    contactPlanFile << maxNodeId - maxSatNodeId << std::endl;

    for (const auto &contact : contacts) {
        if (contact->startTime == contact->endTime
                || contact->startTime > contact->endTime) {
            // we need at least one second of contact, otherwise we'll discard the
            // contact
            continue;
        }
        if (contact->enabled) {
            contactPlanFile << contact->startTime << ";";
            contactPlanFile << contact->endTime << ";";
            contactPlanFile << contact->sourceNodeId << ";";
            contactPlanFile << contact->sinkNodeId << std::endl;
        }
    }
    contactPlanFile.close();
}

}  // namespace estnet
