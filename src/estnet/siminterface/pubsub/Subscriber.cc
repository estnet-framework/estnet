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

#include "Subscriber.h"

#include <omnetpp.h>

namespace estnet {

void Subscriber::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        omnetpp::cObject *details) {
    // try to cast the obj pointer into a sGWValue pointer
    PubSubMsg *pubSubMsg = nullptr;
    try {
        pubSubMsg = omnetpp::check_and_cast<PubSubMsg*>(obj);
    } catch (omnetpp::cRuntimeError &e) {
        throw omnetpp::cRuntimeError(
                "Wrong cObject pointer sent with the signal "
                        "%s! Expected pointer of type sPubSubMsg!\n",
                PUBSUB_SIGNAL_NAME);
    }

    // now check if we need the value...
    if (needsMessage(pubSubMsg->key)) {
        // call the handler of the implementing class
        receivedPubSubMessage(pubSubMsg);
    }
}

void Subscriber::subscribeTopic(std::string topic) {
    _neededMessageKeys.insert((tPubSubKey) topic);
}

bool Subscriber::needsMessage(tPubSubKey messageKey) const {

    //return (_neededMessageKeys.find(messageKey) != _neededMessageKeys.end());
    //EV << "Needed key: \"" << messageKey << "\"." << "\n";

    // iterate all topics we subscribed to
    std::set<tPubSubKey>::iterator it;
    for (it = _neededMessageKeys.begin(); it != _neededMessageKeys.end();
            ++it) {
        tPubSubKey key = *it;

        // if the topics match fully, we can already return true
        if (key == messageKey) {
            return true;
        }

        // else we may match the topic with wildcards, so we now evaluate whether we do or not

        size_t pos_candidate = key.find("/");
        size_t pos_searched = messageKey.find("/");
        bool correct = true;

        // we evaluate the string topic level to topic level, which are separated by /
        // for each level we check whether we are still correct and whether we are still inside the topic string
        while (pos_candidate != std::string::npos
                && pos_searched != std::string::npos && correct) {

            // find next / after the one we already found
            size_t temp_cand = key.find("/", pos_candidate + 1);
            size_t temp_search = messageKey.find("/", pos_searched + 1);

            // get the topiclevel string, so the value between the two /, or till end, if this is the last topic level
            std::string cand_sub = key.substr(pos_candidate + 1,
                    temp_cand == std::string::npos ?
                            std::string::npos : temp_cand - pos_candidate - 1);
            std::string search_sub = messageKey.substr(pos_searched + 1,
                    temp_search == std::string::npos ?
                            std::string::npos : temp_search - pos_searched - 1);

            // if both are either at the end or not at the end, and their topiclevels match, either direct or via a +
            // if our candidate is at its end and its last topiclevel is a #, then we match aswell
            if ((((temp_cand != std::string::npos
                    && temp_search != std::string::npos)
                    || (temp_cand == std::string::npos
                            && temp_search == std::string::npos))
                    && (cand_sub == search_sub || cand_sub == "+"))
                    || (cand_sub == "#" && temp_cand == std::string::npos)) {
                correct = true;
            }
            // if the topiclevel is not okay, we set this to false and the while loop will terminate
            else {
                correct = false;
            }

            pos_candidate = temp_cand;
            pos_searched = temp_search;
        }
        if (correct) {
            return true;
        }
    }

    return false;
}

}  // namespace estnet
