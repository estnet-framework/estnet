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

#include "SerialExternalProtocolModuleBase.h"

#include "estnet/common/ByteHelpers.h"

namespace estnet {

bool SerialExternalProtocolModuleBase::connect(const std::string &port,
        unsigned long baudRate) {
    this->_serial = new serial::Serial(port, baudRate,
            serial::Timeout::simpleTimeout(serial::Timeout::max()));
    return this->_serial->isOpen();
}

void SerialExternalProtocolModuleBase::disconnect() {
    if (this->_serial != nullptr && this->_serial->isOpen()) {
        this->_serial->close();
        delete this->_serial;
        this->_serial = nullptr;
    }
}

void SerialExternalProtocolModuleBase::sendToProtocol(
        const std::string &commandString) {
    // check length of command string
    const uint8_t *commandStringBytes = (const uint8_t*) commandString.c_str();
    size_t commandStringLengthFull = commandString.length();
    if (commandStringLengthFull > UINT16_MAX) {
        throw omnetpp::cRuntimeError("command string longer than 16-bit size");
    }
    uint16_t commandStringLength = commandStringLengthFull;
    // convert length to big endian byte array
    uint8_t lengthBytes[2];
    uint16ToBytes(commandStringLength, lengthBytes);
    // send length and the actual bytes
    this->_serial->write(lengthBytes, 2);
    this->_serial->write(commandStringBytes, commandStringLength);
}

bool SerialExternalProtocolModuleBase::dataFromProtocol() {
    return this->_serial->available() > 0;
}

void SerialExternalProtocolModuleBase::recvFromProtocol(
        std::string &commandString) {
    const size_t bufferSize = UINT16_MAX;
    uint8_t buffer[bufferSize];
    memset(buffer, '\0', bufferSize);
    size_t commandStringLength = 0;
    // receive length of the command string
    this->_serial->read(buffer, 2);
    // convert length from bytes to int
    commandStringLength = bytesToUint16(buffer);
    // receive actual command string
    if (commandStringLength >= bufferSize) {
        throw omnetpp::cRuntimeError("Command string to long for buffer");
    }
    // receive actual command string
    this->_serial->read(buffer, commandStringLength);
    buffer[commandStringLength] = '\0';
    commandString = std::string((char*) buffer);
}

}  // namespace estnet
