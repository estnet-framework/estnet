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

#ifndef PUBSUB_PUBLISHER_H_
#define PUBSUB_PUBLISHER_H_

#include <omnetpp.h>

#include "PubSubMessage.h"

using namespace omnetpp;

namespace estnet {

/**
 * Publisher class.
 * This is to be used if a Module should be enabled to publish to our PubSub system.
 * This inherits from cModule, so it can publish values, therefore you need to replace the inheritance of cModule with
 * an inheritance of Publisher.
 * To be able to publish a signal, the ned of the module also needs to extend the publisher ned.
 **/
class ESTNET_API Publisher: public cModule {
protected:
    virtual void initialize() override;
    virtual void publishValue(tPubSubValue value, tPubSubKey key);
    simsignal_t _valueSigId;
};

}  // namespace estnet

#endif
