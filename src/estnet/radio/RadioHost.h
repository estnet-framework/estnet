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

#ifndef __RADIOS_RADIO_HOST_H__
#define __RADIOS_RADIO_HOST_H__

#include <map>

#include <omnetpp.h>
#include <inet/common/geometry/common/Coord.h>
#include <inet/linklayer/common/MacAddress.h>
#include <inet/physicallayer/contract/packetlevel/IRadio.h>
#include <inet/queueing/contract/IPacketQueue.h>

#include "../node/groundstation/GroundStation.h"
#include "estnet/common/node/NodeRegistry.h"

namespace estnet {

/**
 * Decides which radio a packet needs to be sent with.
 * Deduplicates received packets over multiple radios.
 */
class ESTNET_API RadioHost: public omnetpp::cSimpleModule {
protected:
    /** @brief returns number of initalization stages */
    virtual int numInitStages() const override;
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief message receiver function */
    virtual void handleMessage(omnetpp::cMessage*) override;

    /** @brief checks whether mac address indicates a broadcast */
    virtual bool isBroadcast(const inet::MacAddress &macAddress);
    /** @brief checks whether node no is a connected ground station */
    virtual GroundStation* isForGroundStation(unsigned int destNodeId);
    /** @brief sends Packet to a connected ground station */
    virtual void sendToGroundStation(GroundStation *otherGs, inet::Packet *pkt);
    /** @brief chooses the best radio to sent to destination node no */
    virtual int chooseRadio(unsigned int destNodeId);
private:
    NodeRegistry *_nodeRegistry;
    unsigned int _nodeNo;
    GroundStation *_gs;
    simtime_t _lastSendTime;
    int _msgSameTime;
    inet::units::values::s _interPacketDelay;
    std::map<int, inet::physicallayer::IRadio*> _connectedRadios;
    std::set<std::tuple<unsigned int, long, int64_t>> _seenFrames;
};

}  // namespace estnet

#endif
