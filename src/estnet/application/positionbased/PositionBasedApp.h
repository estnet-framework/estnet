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

#ifndef __APPS__POSITION_BASED_APP_H__
#define __APPS__POSITION_BASED_APP_H__

#include "estnet/application/base/BasicApp.h"
#include "estnet/application/contract/IPositionData.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

/**
 * App that generates data dependent on the position of the
 * containing node over ground
 */
class ESTNET_API PositionBasedApp: public BasicApp {
protected:
    IPositionData *pd;
    virtual ~PositionBasedApp();
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief message receiver function */
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    /** @brief called to figure out when next packet needs to be generated */
    virtual bool nextPacketTime(omnetpp::simtime_t &t);
    /** @brief called to schedule the next packet. */
    virtual void scheduleNextPacket(omnetpp::simtime_t nextTime = -1) override;

private:
    /** gets the multiplier for the current position*/
    virtual double getPositionMultiplier();

    double beamWidth;
    SatMobility *satMobility = nullptr; // SatMobility module as position source
    IEarthModel *earthModel;
    omnetpp::simtime_t _startTime;
    omnetpp::simtime_t _stopTime;
    omnetpp::cMessage *_scheduleMsgEmpty;
};

}  // namespace estnet

#endif
