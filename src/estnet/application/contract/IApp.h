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

#ifndef ESTNET_APPLICATION_CONTRACT_IAPP_H_
#define ESTNET_APPLICATION_CONTRACT_IAPP_H_

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * Interface for apps
 */
class ESTNET_API IApp {
public:
    /**
     * @brief get ID of node that holds this app
     * @return node ID of containing node
     */
    unsigned int getNodeId() const;
    /**
     * @brief get ID of this app
     * @return the app's ID
     */
    unsigned int getId() const;
    /**
     * @brief get number of sent packets
     * @return number of sent packets
     */
    unsigned int getPktSent() const;
    /**
     * @brief get number of received packets
     * @return number of received packets
     */
    unsigned int getPktRcvd() const;

protected:
    unsigned int _nodeId;
    int _id;
    long _numSent = 0;
    long _numReceived = 0;

};

}  // namespace estnet

#endif /* ESTNET_APPLICATION_CONTRACT_IAPP_H_ */
