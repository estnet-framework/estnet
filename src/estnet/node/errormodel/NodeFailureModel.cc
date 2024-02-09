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

#include "NodeFailureModel.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/radio/RadioHost.h"

namespace estnet {

Define_Module(NodeFailureModel);

omnetpp::simsignal_t NodeFailureModel::nodeFailed = registerSignal(
        "nodeFailed");

void NodeFailureModel::initialize() {
    //getting errormodel status
    bool enable = this->par("enable");

    //only start working if enabled
    if (enable) {
        //initialize all the class members
        this->_node = check_and_cast<Satellite*>(this->getParentModule());

        //schedule failure message
        this->_MTTF = this->par("MTTF").doubleValue();
        this->_MTTR = this->par("MTTR").doubleValue();
        this->_faultSeed = this->par("faultSeed");
        _failure = new cMessage("failure");
        _repaired = new cMessage("repaired");
        scheduleAt(exponential(_MTTF, _faultSeed), _failure);

        std::ostringstream strStream;
        strStream << "/omnet/sat/" << _node->getNodeNo() << "/nodefailure";
        _msgKey = strStream.str();

        // initialize publisher
        SimplePublisher::initialize();
    }

}

void NodeFailureModel::handleMessage(cMessage *msg) {
    //checking what kind of message need to be handled
    if (msg == _failure) {
        //node reboots or processes EDACs
        this->emit(nodeFailed, true);
        publishValue(std::to_string(1), _msgKey);
        scheduleAt(simTime() + exponential(_MTTR, _faultSeed), _repaired);
    } else if (msg == _repaired) {
        //node is repaired and ready to send or receive messages
        this->emit(nodeFailed, false);
        publishValue(std::to_string(0), _msgKey);

        scheduleAt(simTime() + exponential(_MTTF, _faultSeed), _failure);
    }
}

}  // namespace estnet
