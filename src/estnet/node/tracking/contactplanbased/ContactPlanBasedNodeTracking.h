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

#ifndef __ESTNET_CONTACTPLANBASEDNODETRACKING_H_
#define __ESTNET_CONTACTPLANBASEDNODETRACKING_H_

#include "estnet/node/tracking/contract/INodeTracking.h"

namespace estnet {

/**
 *  The class ContactPlanBasedNodeTracking is used for setting the orientation to the node that
 *  is given in the global contact plan
 */
class ESTNET_API ContactPlanBasedNodeTracking: public INodeTracking,
        public omnetpp::cListener {
public:
    ContactPlanBasedNodeTracking() :
            _currentlyTrackingNodeNo(0) {

    }
    /** @brief initialization */
    virtual void initialize() override;
    /** @brief cleanup */
    virtual void finish() override;
    /** @brief subscribes to receive position updates */
    virtual void subscribeToSatellitePositions();
    /** @brief unsubscribes from position updates */
    virtual void unsubscribeFromSatellitePositions();
    /**
     * Called to start a tracking
     */
    virtual void start() override;
    /**
     * Called to stop a tracking and turn to default orientation
     */
    virtual void stop() override;
    /**
     * Implementation of the tracking algorithm
     * check which satellite is tracked and return attitude that is
     * pointing toward this satellite
     *
     * @return the target orientation as quaternion
     */
    inet::Quaternion getNewOrientation();

    /** @brief method called with position updates */
    virtual void receiveSignal(cComponent *source, omnetpp::simsignal_t signal,
            cObject *object, cObject *details) override;

protected:
    unsigned int _currentlyTrackingNodeNo;  // Id of node that is tracked

    using omnetpp::cListener::finish; // cListener's finish function is also
                                      // relevant

    /** @brief calculated the relative orientation to current target
     *  @return relative orientation to target */
    virtual inet::Quaternion track();

};

}
//namespace

#endif
