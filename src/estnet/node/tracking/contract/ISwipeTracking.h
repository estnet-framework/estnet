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

#ifndef __ESTNET_ISWIPETRACKING_H_
#define __ESTNET_ISWIPETRACKING_H_

#include "estnet/node/tracking/contract/INodeTracking.h"

namespace estnet {

/**
 *  Interface for tracking algorithms that are based on a swipe through multiple
 *  satellites during a ground station passing of a formation
 *  It provides parameter like total tracking time and delta T between first and last
 *  satellite
 *  The contact start/stop times are extracted from the contact plan
 */
class ESTNET_API ISwipeTracking: public INodeTracking {
public:
    /**
     * This is a abstract function for the concrete tracking algorithm
     * This function calculates a target orientation that the node is
     * supposed to control its attitude to
     *
     * @return the target orientation as quaternion
     */
    virtual inet::Quaternion getNewOrientation() = 0;
protected:
    /** @brief initialization */
    virtual void initialize() override;
    /** @brief scheduled self-messages receiver function */
    virtual void handleMessage(omnetpp::cMessage *message) override;
    /** @brief cleanup messages */
    virtual void finish() override;

    /*
     * sets time parameters of satellite passing like start and stop time,
     * tracking time and delta T
     */
    virtual void updateTimeParameters();

    // ids of satellites that are contained in formation
    std::pair<unsigned int, unsigned int> _trackingNodeIds;

    std::pair<cJulian, cJulian> _T; // start/stop time of swipe

    omnetpp::cMessage *_swipeStart; // event message indicating start of tracking
    omnetpp::cMessage *_swipeStop;  // event message indicating end of tracking
    double _trackingTime;           // total tracking time
    double _deltaT;                 // total tracking time - mean tracking time

    // statistic for indicating which satellite is tracked
    static omnetpp::simsignal_t relTarget;

};

} //namespace

#endif
