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

#ifndef __NODES_CONNECTED_GROUND_STATION_H__
#define __NODES_CONNECTED_GROUND_STATION_H__

#include "estnet/node/base/NodeBase.h"
#include "estnet/mobility/terrestrial/StaticTerrestrialMobility.h"

namespace estnet {

/**
 * represents a ground station
 */
class ESTNET_API GroundStation: public NodeBase {
private:
    std::set<unsigned int> _nodesAvailableToBeTracked;
    inet::Quaternion _defaultOrientation;
    bool _useContactPlanTracking;

protected:
    /** @brief returns number of needed initialization stages */
    virtual int numInitStages() const override;
    /** @brief initialization */
    virtual void initialize(int stage) override;

public:
    GroundStation() :
            _useContactPlanTracking(false) {
    }

    /** @brief Returns the node's mobility */
    virtual StaticTerrestrialMobility* getMobility() const override;
    /** @brief called when a contact gets active */
    virtual void addContactTo(unsigned int other_node_no) override;
    /** @brief called when a contact gets inactive */
    virtual void removeContactTo(unsigned int other_node_no) override;

    /** @brief checks whether this ground station is connected to the given ground station */
    virtual bool canCommunicateWithoutRadioWith(
            unsigned int other_node_no) const;

    /** @brief function only called when the contactplan tracking is used */
    virtual void enableContactPlanTracking();

    /**
     * get all nodes that can be tracked at the current simulation time
     *
     * @return set<unsigned int>: A set of node IDs that are in tracking range
     */
    std::set<unsigned int> nodesAvailableToBeTracked();
};

}  // namespace estnet

#endif
