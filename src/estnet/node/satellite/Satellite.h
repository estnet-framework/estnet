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

#ifndef __NODES_SATELLITE_H__
#define __NODES_SATELLITE_H__

#include "estnet/node/base/NodeBase.h"
#include "estnet/mobility/contract/IExtendedMobility.h"
#include "estnet/attitudecontroller/AttitudeController.h"

namespace estnet {

/**
 * Represents a satellite node in the simulation
 */
class ESTNET_API Satellite: public NodeBase {
protected:
    bool alignmentCheckEnable;
    bool contactTrackingEnable;
    /** @brief returns number of initialization stages */
    virtual int numInitStages() const override;
    /** @brief called for initialization */
    virtual void initialize(int stage) override;

public:
    AttitudeController *attCon;
    double contactTrackingUpdateTime;
    /**
     * Get information about whether the alignment check is enabled
     * This information is required to set the satellite into pointing
     * mode to other nodes
     * @return bool: true if alignment check is enabled
     */
    virtual bool getAlignmentCheckEnable();
    /** @brief returns the nodes mobility */
    virtual IExtendedMobility* getMobility() const override;
    /** @brief called when a contact gets active */
    virtual void addContactTo(unsigned int other_node_no) override;
    /** @brief called when a contact gets inactive */
    virtual void removeContactTo(unsigned int other_node_no) override;
};

}  // namespace estnet

#endif
