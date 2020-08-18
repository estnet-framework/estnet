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

#ifndef __ESTNET_EXPSWIPETRACKING_H_
#define __ESTNET_EXPSWIPETRACKING_H_

#include "estnet/node/tracking/contract/ISwipeTracking.h"

namespace estnet {

/**
 * Swipe algorithm that uses a exponentially increasing/decreasing
 * angular velocity
 */
class ESTNET_API ExponentialSwipeTracking: public ISwipeTracking {
public:
    /**
     * Implementation of the tracking algorithm
     * calculate percentage of passed pass time, use this information
     * to pass through the formation with the given velocity function
     *
     * @return the target orientation as quaternion
     */
    virtual inet::Quaternion getNewOrientation();
protected:
    /** @brief initialization */
    virtual void initialize() override;

    /**
     * exponential function, is a function that rescales the ratio
     * of the already passed ground station pass
     * @param ratio: real ratio of current time by overall pass time
     * @return fake ratio, exonentially scaled
     */
    virtual double applyScaling(double ratio);

    int _exponent;
};

} //namespace

#endif
