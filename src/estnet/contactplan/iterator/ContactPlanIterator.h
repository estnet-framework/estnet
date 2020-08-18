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

#ifndef __CONTACTPLANS_ITERATOR_H__
#define __CONTACTPLANS_ITERATOR_H__

#include <vector>
#include "estnet/contactplan/common/Contacts.h"
#include "estnet/common/queue/PQueue.h"

namespace estnet {

/**
 * compares contacts for a max heap
 */
template<typename T>
class ESTNET_API ContactPlanEntryStartTimeComparatorMinHeap {
public:
    explicit ContactPlanEntryStartTimeComparatorMinHeap() {
    }
    bool operator()(const T *lhs, const T *rhs) const {
        return (lhs->startTime > rhs->startTime);
    }
};

/**
 * compares contacts for a min heap
 */
template<typename T>
class ESTNET_API ContactPlanEntryEndTimeComparatorMinHeap {
public:
    explicit ContactPlanEntryEndTimeComparatorMinHeap() {
    }
    bool operator()(const T *lhs, const T *rhs) const {
        return (lhs->endTime > rhs->endTime);
    }
};

/**
 * Provides an unified interface
 * to iterate over a contact plan
 */
template<typename T>
class ESTNET_API ContactPlanIterator {
public:
    /** @brief initialize with empty contact plan */
    ContactPlanIterator();
    /** @brief initialize with given contact plan */
    ContactPlanIterator(const std::vector<T*> &contacts);
    /** @brief initialize with given contact plan and min & max timesteps to take */
    ContactPlanIterator(const std::vector<T*> &contacts, int64_t minTimestep,
            int64_t maxTimestep);
    /** @brief cleanup */
    ~ContactPlanIterator();
    /** @brief resets the contact plan to iterate over */
    void setContacts(const std::vector<T*> &contacts);
    /** @brief returns the next time the contact plan changes after the given time */
    int64_t getNextChange(int64_t time) const;
    /** @brief iterates over the plan until the given time.
     *  returns the changed contacts from now until the given time */
    std::vector<T*> getChangesUntil(int64_t time);
    /** @brief returns all contacts */
    const std::vector<T*> getAllContacts() const;
    /** @brief returns all currently active contacts */
    const std::vector<T*> getActiveContacts() const;
    /** @brief returns all upcoming contacts */
    const std::vector<T*> getUpcomingContacts() const;

private:
    int64_t _maxTimestep;
    int64_t _minTimestep;
    std::vector<T*> _allContacts;
    pqueue<T*, ContactPlanEntryStartTimeComparatorMinHeap<T>> _upcomingContacts; // sorted by ascending startTime, to figure out next
                                                                                 // active contacts
    pqueue<T*, ContactPlanEntryEndTimeComparatorMinHeap<T>> _activeContacts; // sorted by ascending endTime, to figure out expiring
                                                                             // contacts
};

}  // namespace estnet

#endif
