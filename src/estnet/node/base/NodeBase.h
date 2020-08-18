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

#ifndef __NODEBASE_H__
#define __NODEBASE_H__

#include <vector>

#include <inet/mobility/contract/IMobility.h>

#include "estnet/node/manager/NodeContactManager.h"
#include "estnet/application/contract/IApp.h"

namespace estnet {

/**
 * Common base class for all communicating
 *  nodes in the simulation
 */
class ESTNET_API NodeBase: public omnetpp::cModule {
private:
    omnetpp::simsignal_t uplinkCoverageId;
    omnetpp::simsignal_t downlinkCoverageId;
    NodeContactManager* getNodeContactManagerInternal() const;

protected:
    unsigned int _nodeNo;
    virtual int numInitStages() const override;
    virtual void initialize(int stage) override;
    /** @brief cleanup */
    virtual void finish() override;

public:
    /** @brief returns the node no */
    unsigned int getNodeNo() const {
        return this->_nodeNo;
    }

    /** @brief returns the apps */
    std::vector<IApp*> getApps() const;

    /** @brief Checks whether the node has a node manager */
    virtual bool contactManagementEnabled() const;
    /** @brief Returns the node's node manager */
    virtual NodeContactManager* getNodeContactManager() const;
    /** @brief Returns the node's mobility */
    virtual inet::IMobility* getMobility() const = 0;

    /** @brief called when a contact gets active */
    virtual void addContactTo(unsigned int other_node_no);
    /** @brief called when a contact gets inactive */
    virtual void removeContactTo(unsigned int other_node_no);
    /** @brief called during initialization for all contacts of the node */
    virtual void addContactBetween(time_t startTime, time_t endTime,
            unsigned int node_no, unsigned int other_node_no,
            unsigned int bitrate, unsigned int range);
    /** @brief called when a package is buffered for notification about
     *  when it will be released or if it got abandonded
     *  @deprecated in favor of IProtocolModule */
    virtual void bundleRouteNotification(unsigned int sourceNodeId,
            unsigned int destNodeId, long sequenceNumber,
            unsigned int nextHopNodeId, bool abandoned);
    /** @brief called when packets are expected to be released out of limbo
     *  @deprecated in favor of IProtocolModule */
    virtual void waitForLimboDecisions(bool reset = false);
};

}  // namespace estnet

#endif
