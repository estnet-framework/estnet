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

#ifndef PUBSUB_SUBSCRIBER_H_
#define PUBSUB_SUBSCRIBER_H_

#include <set>
#include <string>

#include <omnetpp.h>

#include "PubSubMessage.h"

namespace estnet {

class ESTNET_API Subscriber: public omnetpp::cListener {
public:
    explicit Subscriber(omnetpp::cComponent *ownerComponent = nullptr) {
        if (ownerComponent == nullptr) {
            // No top level owner component defined -> register listener at simulation
            // top level!
            omnetpp::getSimulation()->getSystemModule()->subscribe(
            PUBSUB_SIGNAL_NAME, this);
        } else {
            // subscribe to the value update signal at the level of the ownerComponent
            // (e.g. a satellite module)
            ownerComponent->subscribe(PUBSUB_SIGNAL_NAME, this);
        }
    }

    /**
     * Returns a list of keys this module subscribed to.
     * @return list of keys needed by this module
     */
    virtual std::set<tPubSubKey> getNeededMessageKeys() const {
        return _neededMessageKeys;
    }

    /**
     * Callback function for received pubsub signal. As pubsub only emits PubSubMsg, it can be assumed
     * that obj always contains one, but it gets checked aswell.
     * @param source Component that emitted the signal
     * @param signalID Signal identification number
     * @param obj cObject pointer, that contains the actual message (must be of type
     * PubSubMsg)
     * @param details not used here, can be safely assumed to be NULL
     */
    virtual void receiveSignal(omnetpp::cComponent *source,
            omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
            omnetpp::cObject *details) override;

protected:
    /**
     * This function is called whenever a new message arrives that has a topic this
     * subscriber subscribed to.
     * This function should be overwritten as handler for newly arrived messages.
     * @param PubSubMsg Newly arrived message, that this subscriber subscribed to.
     * category
     */
    virtual void receivedPubSubMessage(PubSubMsg *pubSubMsg) = 0;

    /**
     * Function to subscribe to new topics.
     * @param string topic Topic to subscribe to.
     */
    void subscribeTopic(std::string topic);
    std::set<tPubSubKey> _neededMessageKeys; ///< list of mandatory value keys needed by this module

private:
    /**
     * Internal function to evaluate, whether the key of a message matches with a topic this subscriber subscribed to.
     * This also implements a wildcard functionality, equal to what MQTT uses, so we are compatible with it.
     *
     */
    bool needsMessage(tPubSubKey msgKey) const;
};

}  // namespace estnet

#endif
