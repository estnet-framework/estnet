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

#ifndef __ATTITUDE_TARGET_H__
#define __ATTITUDE_TARGET_H__

#include <string>

#include "estnet/node/base/NodeBase.h"

namespace estnet {

/**
 * Interface class for different targets of the attitude controller.
 */
class ESTNET_API AttitudeTarget {
private:
    unsigned int target = 0;
    inet::Coord targetCoord;
    bool nilTarget;
    bool earth_CenterTarget;
    bool sunTarget;
    bool nodeNumberTarget;
    bool coordTarget;
    static inet::Coord _sunPosition;   // sun stays at this point
    // initializes the position of the sun as a static point
    static void initSunPos(double sunOrientation = 0);

public:

    /** @brief default constructor initializes as NIL target*/
    AttitudeTarget();
    /**
     * @brief initializes with simulation node as target
     * @param target: node number of the target node
     */
    AttitudeTarget(unsigned int target);
    /**
     * @brief initializes with EARTH_CENTER, SUN, NIL or node number as target
     * @param target: string which is either "EARTH_CENTER", "SUN", "NIL" or a node number
     */
    AttitudeTarget(std::string target);
    /**
     * @brief initializes with fixed coordinates in ECI frame as target
     * @param target: coordinates of the target in ECI frame
     */
    AttitudeTarget(inet::Coord target);
    /**
     * @brief checks if the current target is "EARTH_CENTER"
     */
    virtual bool isEarth_Center() const;
    /**
     * @brief checks if the current target is "NIL"
     */
    virtual bool isNil() const;
    /**
     * @brief checks if the current target is "SUN"
     */
    virtual bool isSun() const;
    /**
     * @brief checks if the current target node number
     */
    virtual bool isNodeNumber() const;
    /**
     * @brief checks if the current target node number
     * @param nodeNumber: if the current target is a node number pass node number into parameter
     */
    virtual bool isNodeNumber(unsigned int &nodeNumber) const;
    /**
     * @brief checks if the current target is given as coordinates
     */
    virtual bool isCoord() const;
    /**
     * @brief gets the coordinates of any target type in ECI frame
     */
    virtual void getTargetCoord(inet::Coord &targetCoord);
    /**
     * @brief gets the coordinates of any target type in ECI frame
     */
    virtual inet::Coord getTargetCoord();
    /**
     * @brief gets the coordinates of any target type in ECI frame at a specific time point
     * @param time: time since the start of the simulation
     */
    virtual void getTargetCoordAtTime(inet::Coord &targetCoord, double time);
    /**
     * @brief gets the velocity of any target type in ECI frame
     */
    virtual void getTargetVelocity(inet::Coord &targetVelocity);
    /**
     * @brief gets the velocity of any target type in ECI frame at a specific time point
     * @param time: time since the start of the simulation
     */
    virtual void getTargetVelocityAtTime(inet::Coord &targetVelocity,
            double time);
    friend std::ostream& operator<<(std::ostream &os,
            const AttitudeTarget &target);
};

/** @brief Override print operator to print current target */
inline std::ostream& operator<<(std::ostream &os,
        const AttitudeTarget &target) {
    os << "Attitude Target: ";
    if (target.isEarth_Center())
        os << "EARTH_CENTER";
    else if (target.isNil())
        os << "NIL";
    else if (target.isSun())
        os << "SUN";
    else if (target.isCoord())
        os << "Coord: " << target.coordTarget;
    else
        os << "Node Number: " << target.target;
    return os;
}

}  // namespace estnet

#endif
