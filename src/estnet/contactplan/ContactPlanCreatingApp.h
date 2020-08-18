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

#ifndef __OMNET_ION__APP_CP_H__
#define __OMNET_ION__APP_CP_H__

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * An app used to generate data during
 * contact plan creation.
 * Strictly not needed anymore, as the
 * ~ContactPlanCreatingRadioMedium fakes
 * its own traffic, but kept for debugging
 * purposes.
 */
class ESTNET_API ContactPlanCreatingApp: public omnetpp::cSimpleModule {
protected:
    /** @brief cancel and delete messages */
    virtual ~ContactPlanCreatingApp();
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief returns number of initialization stages */
    virtual int numInitStages() const override;
    /** @brief scheduled self-messages receiver function */
    virtual void handleMessage(omnetpp::cMessage*) override;
    /** @brief cleanup & plan writing */
    virtual void finish() override;

private:
    unsigned int _nodeId;
    unsigned long _numReceived;
    omnetpp::simtime_t _startTime;
    omnetpp::simtime_t _stopTime;
    omnetpp::cMessage *_scheduleMsg;
};

}  // namespace estnet

#endif
