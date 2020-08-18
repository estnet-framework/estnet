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

#include "TcpExternalProtocolModuleBase.h"

#include "estnet/protocol/common/SocketUtils.h"

namespace estnet {

bool TcpExternalProtocolModuleBase::connect(const std::string &host, int port) {
    this->_sockfd = open_tcp_client_socket(host.c_str(), port);
    if (this->_sockfd < 0) {
        return false;
    }
    return true;
}

void TcpExternalProtocolModuleBase::disconnect() {
    close_socket(this->_sockfd);
    this->_sockfd = -1;
}

void TcpExternalProtocolModuleBase::sendToProtocol(
        const std::string &commandString) {
    // check length of command string
    const char *commandStringBytes = commandString.c_str();
    size_t commandStringLengthFull = commandString.length();
    if (commandStringLengthFull > UINT16_MAX) {
        throw omnetpp::cRuntimeError("command string longer than 16-bit size");
    }
    uint16_t commandStringLength = commandStringLengthFull;
    // convert length to big endian byte array
    char lengthBytes[2];
    lengthBytes[0] = (commandStringLength >> 8) & 0xFF;
    lengthBytes[1] = (commandStringLength >> 0) & 0xFF;
    // send length and the actual bytes
    sendBytes(this->_sockfd, 2, lengthBytes);
    sendBytes(this->_sockfd, commandStringLength, commandStringBytes);
}

bool TcpExternalProtocolModuleBase::dataFromProtocol() {
    return checkIfDataAvailable(this->_sockfd) > 0;
}

void TcpExternalProtocolModuleBase::recvFromProtocol(
        std::string &commandString) {
    const size_t bufferSize = UINT16_MAX;
    char buffer[bufferSize];
    memset(buffer, '\0', bufferSize);
    size_t commandStringLength = 0;
    // receive length of the command string
    recvBytes(this->_sockfd, 2, buffer);
    // convert length from bytes to int
    commandStringLength += ((buffer[0] & 0xFF) << 8);
    commandStringLength += ((buffer[1] & 0xFF) << 0);
    // receive actual command string
    if (commandStringLength >= bufferSize) {
        throw omnetpp::cRuntimeError("Command string to long for buffer");
    }
    // receive actual command string
    memset(buffer, '\0', bufferSize);
    recvBytes(this->_sockfd, commandStringLength, buffer);
    commandString = std::string(buffer);
}

}  // namespace estnet
