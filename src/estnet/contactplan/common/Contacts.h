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

#ifndef __CONTACTPLANS_CONTACTS_H__
#define __CONTACTPLANS_CONTACTS_H__

#include <stdint.h>
#include <tuple> // std::tie
#include <map>
#include <set>
#include <vector>
#include <algorithm>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

using TimeRange = std::tuple<int64_t, int64_t>;

/**
 * Represents one contact plan entry
 */
typedef struct contact_plan_entry {
    unsigned int sourceNodeId;
    unsigned int sinkNodeId;
    bool sourceIsGroundStation;
    bool sinkIsGroundStation;
    int64_t startTime;
    int64_t endTime;
    int64_t bitrate;
    int64_t range;
    double minDistance;
    double maxDistance;
    bool enabled;
    int64_t maxSimTime;
    contact_plan_entry() {
    }
    contact_plan_entry(const contact_plan_entry &e) :
            sourceNodeId(e.sourceNodeId), sinkNodeId(e.sinkNodeId), sourceIsGroundStation(
                    e.sourceIsGroundStation), sinkIsGroundStation(
                    e.sinkIsGroundStation), startTime(e.startTime), endTime(
                    e.endTime), bitrate(e.bitrate), range(e.range), minDistance(
                    e.minDistance), maxDistance(e.maxDistance), enabled(
                    e.enabled), maxSimTime(e.maxSimTime) {
    }
    bool operator==(const contact_plan_entry &other) {
        return (sourceNodeId == other.sourceNodeId)
                && (sinkNodeId == other.sinkNodeId);
    }
    bool operator!=(const contact_plan_entry &other) {
        return !(*this == other);
    }
} contact_plan_entry_t;

/**
 * Represents one interference plan entry
 */
typedef struct interference_plan_entry {
    unsigned int sourceNodeId;
    unsigned int sinkNodeId;
    int64_t startTime;
    int64_t endTime;
    bool enabled;
    std::vector<unsigned int> interferingNodeId;
} interference_plan_entry_t;

struct timerange_comp {
    bool operator()(const TimeRange &a, const TimeRange &b) {
        return std::get<0>(a) < std::get<0>(b);
    }
};

// convenience names for commonly used types
using ContactPlan = std::vector<contact_plan_entry_t *>;
using InterferencePlan = std::vector<interference_plan_entry_t *>;

/**
 * comparator struct that sorts contacts using the source and sink node id
 */
struct simple_cp_comp {
    bool operator()(const contact_plan_entry_t &l,
            const contact_plan_entry_t &r) {
        auto a = std::tie(l.sourceNodeId, l.sinkNodeId);
        auto b = std::tie(r.sourceNodeId, r.sinkNodeId);
        return a < b;
    }
    bool operator()(const contact_plan_entry_t *l,
            const contact_plan_entry_t *r) {
        return this->operator()(*l, *r);
    }
};

using IndependentSet = std::set<contact_plan_entry_t*, simple_cp_comp>;

/**
 * comparator struct that sorts independentsets
 * using the size first, then the source and sink node id of the contacts
 */
struct independentset_comp {
    bool operator()(IndependentSet a, IndependentSet b) {
        if (a.size() != b.size()) {
            return a.size() < b.size();
        } else {

            auto cmp = [](const contact_plan_entry_t *l,
                    const contact_plan_entry_t *r) {
                auto a = std::tie(l->sourceNodeId, l->sinkNodeId);
                auto b = std::tie(r->sourceNodeId, r->sinkNodeId);
                return a < b;
            };

            auto l = a.begin();
            auto r = b.begin();
            while (l != a.end()) {
                if (*l != *r) {
                    return cmp(*l, *r);
                }
                l++;
                r++;
            }
        }
        return false;
    }
};

using IndependentSetsInTimeRange = std::vector<IndependentSet>;
using IndependentSets = std::map<TimeRange, IndependentSetsInTimeRange, timerange_comp>;

/**
 * Sorts a contact plan by nodes firstm then time
 */
class ESTNET_API ContactPlanEntryNodeFirstComparator {
private:
    const bool _reversed;

public:
    explicit ContactPlanEntryNodeFirstComparator(const bool &revparam = false) :
            _reversed(revparam) {
    }
    /** @brief pointer comparison */
    bool operator()(const contact_plan_entry *lhs,
            const contact_plan_entry *rhs) const {
        auto a = std::tie(lhs->sourceNodeId, lhs->sinkNodeId, lhs->startTime,
                lhs->endTime);
        auto b = std::tie(rhs->sourceNodeId, rhs->sinkNodeId, rhs->startTime,
                rhs->endTime);
        if (_reversed) {
            return a > b;
        }
        return a < b;
    }
    /** @brief reference comparison */
    bool operator()(const contact_plan_entry &lhs,
            const contact_plan_entry &rhs) const {
        return this->operator()(&lhs, &rhs);
    }
};

/**
 * Sorts a contact plan by time first, then nodes
 */
class ESTNET_API ContactPlanEntryTimeFirstComparator {
private:
    const bool _reversed;

public:
    explicit ContactPlanEntryTimeFirstComparator(const bool &revparam = false) :
            _reversed(revparam) {
    }
    /** @brief pointer comparison */
    bool operator()(const contact_plan_entry *lhs,
            const contact_plan_entry *rhs) const {
        auto a = std::tie(lhs->startTime, lhs->endTime, lhs->sourceNodeId,
                lhs->sinkNodeId);
        auto b = std::tie(rhs->startTime, rhs->endTime, rhs->sourceNodeId,
                rhs->sinkNodeId);
        if (_reversed) {
            return a > b;
        }
        return a < b;
    }
    /** @brief reference comparison */
    bool operator()(const contact_plan_entry &lhs,
            const contact_plan_entry &rhs) const {
        return this->operator()(&lhs, &rhs);
    }
};

/**
 * Sorts an interference plan by time first, then nodes
 */
class ESTNET_API InterferencePlanEntryTimeFirstComparator {
private:
    const bool _reversed;

public:
    explicit InterferencePlanEntryTimeFirstComparator(const bool &revparam =
            false) :
            _reversed(revparam) {
    }
    /** @brief pointer comparison */
    bool operator()(const interference_plan_entry_t *lhs,
            const interference_plan_entry_t *rhs) const {
        auto a = std::tie(lhs->startTime, lhs->endTime, lhs->interferingNodeId,
                lhs->sourceNodeId, lhs->sinkNodeId);
        auto b = std::tie(rhs->startTime, rhs->endTime, rhs->interferingNodeId,
                rhs->sourceNodeId, rhs->sinkNodeId);
        if (_reversed) {
            return a > b;
        }
        return a < b;
    }
    /** @brief reference comparison */
    bool operator()(const interference_plan_entry_t &lhs,
            const interference_plan_entry_t &rhs) const {
        return this->operator()(&lhs, &rhs);
    }
};

/**
 * Sorts an interference plan by nodes first, then time
 */
class ESTNET_API InterferencePlanEntryNodeFirstComparator {
private:
    const bool _reversed;

public:
    explicit InterferencePlanEntryNodeFirstComparator(const bool &revparam =
            false) :
            _reversed(revparam) {
    }
    /** @brief pointer comparison */
    bool operator()(const interference_plan_entry_t *lhs,
            const interference_plan_entry_t *rhs) const {
        auto a = std::tie(lhs->interferingNodeId, lhs->sourceNodeId,
                lhs->sinkNodeId, lhs->startTime, lhs->endTime);
        auto b = std::tie(rhs->interferingNodeId, rhs->sourceNodeId,
                rhs->sinkNodeId, rhs->startTime, rhs->endTime);
        if (_reversed) {
            return a > b;
        }
        return a < b;
    }
    /** @brief reference comparison */
    bool operator()(const interference_plan_entry_t &lhs,
            const interference_plan_entry_t &rhs) const {
        return this->operator()(&lhs, &rhs);
    }
};

}  // namespace estnet

#endif
