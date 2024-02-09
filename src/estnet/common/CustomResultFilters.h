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

#ifndef __UTILS_CUSTOM_RESULT_FILTERS_H__
#define __UTILS_CUSTOM_RESULT_FILTERS_H__

#include "ESTNETDefs.h"

namespace estnet {

/** Result filter that returns the number 1
 */
class ESTNET_API OneFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns the number -1
 */
class ESTNET_API MinusOneFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns the negative byte length of a cPacket
 */
class ESTNET_API NegativePacketBytesFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns number of hops
 * of a packet
 */
class ESTNET_API PacketNumHopsFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns the sequence number
 * of a packet
 */
class ESTNET_API PacketSequenceNumberFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns the source node no
 * of a packet
 */
class ESTNET_API PacketSourceNodeNoFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns the destination node no
 * of a packet
 */
class ESTNET_API PacketDestinationNodeNoFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that returns the next hops node no
 * of a packet
 */
class ESTNET_API NextHopNodeNoFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};


/** Result filter that returns the source node no
 * of an encapsulated Packet
 */
class ESTNET_API MacFrameSourceNodeNoFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};
/** Result filter that returns the destination node no
 * of an encapsulated Packet
 */
class ESTNET_API MacFrameDestinationNodeNoFilter: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};

/** Result filter that checks if a packet is dropped in the mac module due to bit errors
 * is addressed to the current satellite or not. If it is not for this satellite this packet
 * is not relevant for the analysis of packet collisions. If the packet is for this satellite
 * and is dropped due to bit error, a collision of two or more packets may has happened and is recorded. */
class ESTNET_API PacketDroppedDueToBitErrorOrCollision: public omnetpp::cObjectResultFilter {
public:
    using omnetpp::cObjectResultFilter::receiveSignal;
    virtual void receiveSignal(omnetpp::cResultFilter *prev,
            omnetpp::simtime_t_cref t, omnetpp::cObject *object,
            omnetpp::cObject *details) override;
};


}  // namespace estnet

#endif
