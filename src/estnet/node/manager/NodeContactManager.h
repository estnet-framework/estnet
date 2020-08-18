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

#ifndef __NODES_NODE_CONTACT_MANAGER_H__
#define __NODES_NODE_CONTACT_MANAGER_H__

#include <ctime>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/** Receives notifications about available contacts
 *  from the simulations contact plan manager.
 *  @deprecated in favor of IProtocolModule
 */
class ESTNET_API NodeContactManager {
public:
    /** @brief called when a contact gets active */
    virtual void addContactTo(unsigned int other_node_no) = 0;
    /** @brief called when a contact gets inactive */
    virtual void removeContactTo(unsigned int other_node_no) = 0;
    /** @brief called during initialization for all contacts of the node */
    virtual void addContactBetween(time_t startTime, time_t endTime,
            unsigned int node_no, unsigned int other_node_no,
            unsigned int bitrate, unsigned int range) = 0;
    /** @brief called when a package is buffered for notification about
     *  when it will be released or if it got abandonded
     *  @deprecated in favor of IProtocolModule */
    virtual void updateSendingTimeout(unsigned int sourceNodeId,
            long sequenceNumber, unsigned int nextHopNodeId,
            bool abandoned) = 0;
    /** @brief called when packets are expected to be released out of limbo
     *  @deprecated in favor of IProtocolModule */
    virtual void waitForLimboDecisions(bool reset = false) = 0;
};

}  // namespace estnet

#endif
