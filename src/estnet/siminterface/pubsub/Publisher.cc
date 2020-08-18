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

#include "Publisher.h"

#include <omnetpp.h>

namespace estnet {

void Publisher::initialize() {
    //register the publisher to the global signal name
    _valueSigId = registerSignal(PUBSUB_SIGNAL_NAME);
    EV << "publisher" << std::endl;
}

void Publisher::publishValue(tPubSubValue value, tPubSubKey key) {
    PubSubMsg msg;
    if (key.compare("") == 0) {
        throw omnetpp::cRuntimeError(
                "Cannot publish values to an empty topic.");
    }
    msg.key = key;
    msg.value = value;
    // publish the message to the global signal
    emit(_valueSigId, &msg);
}

}  // namespace estnet
