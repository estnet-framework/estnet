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

#ifndef __ESTNET_WHEIGHTEDSWIPETRACKING_H_
#define __ESTNET_WHEIGHTEDSWIPETRACKING_H_

#include "estnet/node/tracking/contract/ISwipeTracking.h"

namespace estnet {

/**
 * Swipe algorithm that uses a weighting function
 * that can be used to reschedule the tracking
 */
class ESTNET_API WeightedSwipeTracking: public ISwipeTracking {
public:
    /*
     * The swipe is rescheduled to a certain part of a ground station pass
     * depending on the given schedule, the time of each satellite is wheighted
     * @param schedule: contains a mapping of node id to fraction of total swipe time
     * @param ratioOfPassTime: the ration of the next pass time that is scheduled, that
     *                          shall be used as deltaT for the given schedule
     */
    void reschedule(std::map<unsigned int, double> schedule,
            double ratioOfPassTime = 1);
    /**
     * Implementation of the tracking algorithm
     * calculate percentage of passed pass time, use this information
     * to pass through the formation with the given velocity function
     *
     * @return the target orientation as quaternion
     */
    inet::Quaternion getNewOrientation() override;

protected:
    /** @brief scheduled self-messages receiver function */
    virtual void handleMessage(omnetpp::cMessage *message) override;

    /*
     * sets time parameters of satellite passing like start and stop time,
     * tracking time and delta T
     */
    void updateTimeParametersRescheduled(bool startReschedule);

    omnetpp::cMessage *_endOfReschedule; // event message indicating end of rescheduled tracking
    bool _scheduled;                            // scheduling state
    std::map<unsigned int, double> _schedule; // schedule weights for each satellites
    double _ratioOfPassTime;        // save ratio of pass time used for schedule

};

} //namespace

#endif
