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

#ifndef __ESTNET_MEANSWIPETRACKING_H_
#define __ESTNET_MEANSWIPETRACKING_H_

#include "estnet/node/tracking/contract/INodeTracking.h"

namespace estnet {

/**
 * Tracking based on a mean vector to all satellites,
 * where each satellite is weighted exponentially with its
 * distance to the ground station
 */
class ESTNET_API MeanTracking: public INodeTracking {

public:
    /**
     * Implementation of the tracking algorithm
     * Calculates the weighted mean of all satellites that are available
     * Determines orientation to this weighted mean
     *
     * @return the target orientation as quaternion
     */
    inet::Quaternion getNewOrientation();

protected:
    /** @brief initialization */
    virtual void initialize() override;
    /** @brief scheduled self-messages receiver function */
    virtual void handleMessage(omnetpp::cMessage *message) override;
    /** @brief cleanup messages */
    virtual void finish() override;

    // ids of satellites that are contained in formation
    std::pair<unsigned int, unsigned int> _trackingNodeIds;
    std::pair<cJulian, cJulian> _T; // start/stop time of swipe

    omnetpp::cMessage *_swipeStart; // event message indicating start of tracking
    omnetpp::cMessage *_swipeStop;  // event message indicating end of tracking
    double _trackingTime;           // total tracking time
    double _deltaT;                 // total tracking time - mean tracking time
    int _exponent;                  // exponent used for weighting

    // statistic for indicating which satellite is tracked
    static omnetpp::simsignal_t relTarget;
};

} //namespace

#endif
