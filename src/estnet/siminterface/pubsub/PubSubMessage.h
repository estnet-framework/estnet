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

#ifndef PUBSUB_MESSAGE_H_
#define PUBSUB_MESSAGE_H_

#define PUBSUB_SIGNAL_NAME                                                       \
  "PubSubSignal" ///< Global name of the signal, where all messages for PubSub will be published

#include "estnet/common/ESTNETDefs.h"
#include <string>

namespace estnet {

// type for the key of a PubSub-Message
typedef std::string tPubSubKey;
// type for the value of a PubSub-Message
typedef std::string tPubSubValue;

/**
 * This represents the object which gets emitted on the signal. Subscribers will get the message and check if they need it.
 * They check the key value. If they want the message, they can use the value provided.
 **/
class ESTNET_API PubSubMsg: public omnetpp::cObject {
public:
    tPubSubKey key;
    tPubSubValue value;
};

}  // namespace estnet

#endif
