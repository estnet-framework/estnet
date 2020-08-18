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

#ifndef __PROTOCOLS__SERIAL_EXTERNAL_PROTOCOL_MODULE_BASE_H__
#define __PROTOCOLS__SERIAL_EXTERNAL_PROTOCOL_MODULE_BASE_H__

#include <serial/serial.h>

#include "ExternalProtocolModuleBase.h"

namespace estnet {

/**
 * Implementation of ~ExternalProtocolModuleBase, which
 * uses a serial port for communication with an external
 * process
 */
class ESTNET_API SerialExternalProtocolModuleBase: public ExternalProtocolModuleBase {
protected:
    /** @brief connects to the external process */
    virtual bool connect(const std::string &port, unsigned long baudRate);
    /** @brief disconnects from the external process */
    virtual void disconnect();
    /** @brief checks whether the serial port has data to receive available */
    virtual bool dataFromProtocol() override;
    /** @brief sends the given bytes string over the serial port */
    virtual void sendToProtocol(const std::string &commandString) override;
    /** @brief receives a byte string from the serial port */
    virtual void recvFromProtocol(std::string &commandString) override;
private:
    serial::Serial *_serial;
};

}  // namespace estnet

#endif
