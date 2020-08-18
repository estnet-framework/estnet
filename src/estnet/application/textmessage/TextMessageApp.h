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

#ifndef __ESTNET_TEXTMESSAGEAPP_H_
#define __ESTNET_TEXTMESSAGEAPP_H_

#include <string>

#include "estnet/application/base/BasicApp.h"
#include "estnet/application/contract/IPositionData.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

/*
 * An app that sends text messages as packet payload
 * @see TextMessageApp.ned
 */
class ESTNET_API TextMessageApp: public BasicApp {
public:
    TextMessageApp();

protected:
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief cleanup function */
    virtual void finish() override;

    /** @brief internal method to receive packets */
    virtual void recvPacketInternal(omnetpp::cMessage *msg) override;
    /** @brief called to set payload of generated packet */
    virtual inet::Packet* generatePacketPayload() override;

    std::string _textMessage;
    size_t _wrongTextMessagesRcvd;
};

}  // namespace estnet

#endif
