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

#ifndef __PROTOCOLS__DIRECT_PROTOCOL_MODULE_BASE_H__
#define __PROTOCOLS__DIRECT_PROTOCOL_MODULE_BASE_H__

#include "estnet/protocol/base/ProtocolModuleBase.h"
#include "estnet/application/common/AppHostHeader_m.h"
#include "estnet/protocol/common/NextHopTag_m.h"

namespace estnet {

/**
 * Implementation ~ProtocolModuleBase for the simple protocol.
 * Just converts received app packets to radio frames and
 * received radio frames to app packets.
 */
class ESTNET_API DirectProtocolModuleBase: public ProtocolModuleBase {
protected:
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief checks whether a response is ready */
    virtual bool dataFromProtocol() override;
    /** @brief receives response from protocol */
    virtual omnetpp::cPacket* recvFromProtocol() override;
    /** @brief receives new packet for protocol */
    virtual void processPacketFromUpperLayer(inet::Packet *appPacket) override;
    /** @brief receives new frame for protocol */
    virtual void processPacketFromLowerLayer(inet::Packet *appPacket) override;
private:
    omnetpp::cPacket *response = nullptr;
    bool useBroadcasts;
};

}  // namespace estnet

#endif
