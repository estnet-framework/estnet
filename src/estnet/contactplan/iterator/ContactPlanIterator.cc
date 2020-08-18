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

#include "ContactPlanIterator.h"

namespace estnet {

template<typename T>
ContactPlanIterator<T>::ContactPlanIterator() :
        ContactPlanIterator(std::vector<T*>()) {
}

template<typename T>
ContactPlanIterator<T>::ContactPlanIterator(const std::vector<T*> &contacts) :
        ContactPlanIterator(contacts, -1, -1) {
}

template<typename T>
ContactPlanIterator<T>::ContactPlanIterator(const std::vector<T*> &contacts,
        int64_t minTimestep, int64_t maxTimestep) {
    this->setContacts(contacts);
    this->_maxTimestep = maxTimestep;
    this->_minTimestep = minTimestep;
}

template<typename T>
ContactPlanIterator<T>::~ContactPlanIterator() {
    this->_upcomingContacts.clear();
    this->_activeContacts.clear();
}

template<typename T>
void ContactPlanIterator<T>::setContacts(const std::vector<T*> &contacts) {
    for (auto contact : contacts) {
        this->_upcomingContacts.push(contact);
        this->_allContacts.push_back(contact);
    }
}

template<typename T>
int64_t ContactPlanIterator<T>::getNextChange(int64_t time) const {
    // figure out when the next change is
    int64_t nextChange = -1;
    if (this->_activeContacts.empty() && this->_upcomingContacts.empty()) {
        // no new or expiring contacts, not doing anything
        nextChange = -1;
    } else if (!this->_activeContacts.empty()
            && this->_upcomingContacts.empty()) {
        // only expiring contacts
        nextChange = this->_activeContacts.top()->endTime;
    } else if (this->_activeContacts.empty()
            && !this->_upcomingContacts.empty()) {
        // only new contacts
        nextChange = this->_upcomingContacts.top()->startTime;
    } else if (!this->_activeContacts.empty()
            && !this->_upcomingContacts.empty()) {
        // new & expiring contacts
        int64_t nextExpiringContactTime = this->_activeContacts.top()->endTime;
        int64_t nextNewContactTime = this->_upcomingContacts.top()->startTime;
        nextChange = std::min(nextNewContactTime, nextExpiringContactTime);
    }

    // if minTimestep or maxTimestep is set, enforce it
    if (this->_minTimestep > -1 && nextChange > -1) {
        nextChange = std::max(time + this->_minTimestep, nextChange);
    }
    if (this->_maxTimestep > -1 && nextChange > -1) {
        nextChange = std::min(time + this->_maxTimestep, nextChange);
    }
    return nextChange;
}

template<typename T>
std::vector<T*> ContactPlanIterator<T>::getChangesUntil(int64_t time) {
    std::vector<T*> changes;

    // check for expired contacts
    while (!this->_activeContacts.empty()
            && this->_activeContacts.top()->endTime <= time) {
        T *oldContact = this->_activeContacts.pop();
        changes.push_back(oldContact);
    }

    // check for new contacts
    while (!this->_upcomingContacts.empty()
            && this->_upcomingContacts.top()->startTime <= time) {
        T *newContact = this->_upcomingContacts.pop();
        this->_activeContacts.push(newContact);
        changes.push_back(newContact);
    }

    return changes;
}

template<typename T>
const std::vector<T*> ContactPlanIterator<T>::getAllContacts() const {
    return this->_allContacts;
}

template<typename T>
const std::vector<T*> ContactPlanIterator<T>::getActiveContacts() const {
    return this->_activeContacts.copy();
}

template<typename T>
const std::vector<T*> ContactPlanIterator<T>::getUpcomingContacts() const {
    return this->_upcomingContacts.copy();
}

// explicit template instantiation
template class ContactPlanIterator<contact_plan_entry_t> ;
template class ContactPlanIterator<interference_plan_entry_t> ;

}  // namespace estnet
