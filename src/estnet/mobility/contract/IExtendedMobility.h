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

#ifndef __I_EXTENDED_MOBILITY_H__
#define __I_EXTENDED_MOBILITY_H__

#include <inet/mobility/contract/IMobility.h>

#include "estnet/environment/contract/IEarthModel.h"
#include "estnet/common/matrix/Matrix.h"

namespace estnet {

/** Extension to Inet's IMobility to allow accurate
 *  Visualization and simulation of orientation.
 */
class ESTNET_API IExtendedMobility: public inet::IMobility {
public:
    IExtendedMobility();
    virtual ~IExtendedMobility();

    /** @brief Returns the rotation matrix for the current attitude relative to
     *  OSGs ENU coordinate frame */
    virtual M4x4d getCurrentAngularPositionRelativeToENU(cJulian time,
            const inet::Coord &pos);
    /** @brief Returns the rotation euler angles from world orientation to ENU */
    virtual inet::EulerAngles getWorldToEnuAngles(cJulian time,
            const inet::Coord &p);
    /**
     * Returns the current position at the given simulation time (e.g. to create a
     * orbit-circle in OSG-Canvas)
     * @param time target simulation time in seconds
     * @return Position at given simulation time
     */
    virtual inet::Coord getPositionAtTime(double time) = 0;

protected:
    /** @brief Returns the rotation matrix from ENU orientation to world */
    virtual void getEnuToWorldMatrix(cJulian time, const inet::Coord &p,
            double matrix[3][3]);
    /** @brief Returns the rotation matrix from world orientation to ENU */
    virtual void getWorldToEnuMatrix(cJulian time, const inet::Coord &p,
            double matrix[3][3]);
    const GlobalJulianDate *_jdGlobal;
private:
    IEarthModel *_osgEarthModel;

};

}  // namespace estnet

#endif
