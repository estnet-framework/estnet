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

#ifndef __PROTOCOLS__TCP_EXTERNAL_PROTOCOL_MODULE_BASE_H__
#define __PROTOCOLS__TCP_EXTERNAL_PROTOCOL_MODULE_BASE_H__

#include "ExternalProtocolModuleBase.h"

namespace estnet {

/**
 * Implementation of ~ExternalProtocolModuleBase, which
 * uses a TCP socket for communication with an external
 * process
 */
class ESTNET_API TcpExternalProtocolModuleBase: public ExternalProtocolModuleBase {
protected:
    /** @brief connects to the external process */
    virtual bool connect(const std::string &host, int port);
    /** @brief disconnects from the external process */
    virtual void disconnect();
    /** @brief checks whether the socket has data to receive available */
    virtual bool dataFromProtocol() override;
    /** @brief sends the given bytes string over the socket */
    virtual void sendToProtocol(const std::string &commandString) override;
    /** @brief receives a byte string from the socket */
    virtual void recvFromProtocol(std::string &commandString) override;
private:
    int _sockfd;
};

}  // namespace estnet

#endif
