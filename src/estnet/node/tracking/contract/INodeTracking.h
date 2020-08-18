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

#ifndef __ESTNET_INODETRACKING_H_
#define __ESTNET_INODETRACKING_H_

#include <inet/common/geometry/common/Quaternion.h>
#include "../../groundstation/GroundStation.h"

#include "estnet/common/ESTNETDefs.h"
#include "estnet/common/time/GlobalJulianDate.h"

namespace estnet {

/**
 * Interface for tracking classes
 * Provides abstraction for implementation of satellite tracking algorithms
 */
class ESTNET_API INodeTracking: public omnetpp::cSimpleModule {
public:
    /**
     * The function is called by the mobility module in order to update
     * its attitude to the requested attitude  by the tracking algorithm
     * This function call getNewOrientation() of the concrete implementation
     * that holds the concrete algorithm
     */
    virtual void updateAttitude();

    /**
     * Called to start a tracking
     */
    virtual void start();

    /**
     * Called to stop a tracking and turn to default orientation
     */
    virtual void stop();
    /**
     * Get information about the tracking state
     * @return true when tracking was started, false if it is not
     * tracking at the moment
     */
    virtual bool isTracking() const;
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
    virtual void initialize();

    /**
     * @brief turns ground station to default orientation (pointing to zenith)
     * @return the target orientation as quaternion
     */
    virtual inet::Quaternion turnToDefault();

private:
    cJulian _lastUpdate;        // time of last attitude calculation
    inetu::s _updateInterval;  // minimum time between two attitude calculations
    bool _isTracking;           // tracking state of tracking module
    bool _enabled; // overall enable of tracking module, only works when enabled
};

} //namespace

#endif
