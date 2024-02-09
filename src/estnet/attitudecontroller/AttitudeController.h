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

#ifndef __ATTITUDE_CONTROLLER_H__
#define __ATTITUDE_CONTROLLER_H__

#include "AttitudeController.h"

#include <inet/common/Units.h>

#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/power/battery/SimpleEpBattery.h"
#include "AttitudeTarget.h"

namespace estnet {

/**
 * Basic attitude controller for satellites that also calculates the current power needed to perform a maneuver.
 */
class ESTNET_API AttitudeController: public omnetpp::cSimpleModule,
        public inet::power::IEpEnergyConsumer {
private:
    double maxAngularAcceleration[3];
    inet::power::W _maxPower;
    inet::power::W _idlePower;
    cJulian _lastUpdateTime;
    inet::power::W _powerConsumption = inet::power::W(0);
    AttitudeTarget target;
    double targetUpdateTime;
    double recordingUpdateTime;
    inet::Coord pointingAxis;
    omnetpp::cMessage *attitudeControllerUpdate;
    omnetpp::cMessage *recordingUpdate;
    SimpleEpBattery *_energySource = nullptr;

    static omnetpp::simsignal_t accA;
    static omnetpp::simsignal_t accB;
    static omnetpp::simsignal_t accG;
    static omnetpp::simsignal_t pointingError;

    // returns the mobility module of this satellite
    virtual SatMobility* getMobility() const;
    virtual void updateAttitude();
    virtual void calcAttitudeChange();

    virtual void recordePointingError();

protected:
    /** @brief returns number of initialize stages  */
    virtual int numInitStages() const override
    {
        return inet::InitStages::NUM_INIT_STAGES;
    }
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief message receiver function */
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    /** @brief finish */
    virtual void finish() override;

public:
    virtual ~AttitudeController();
    /** @brief Changes the target which the satellite is pointing to */
    virtual void changeTarget(AttitudeTarget &newTarget,
            double targetUpdateTime = 60.0, inet::Coord pointingAxis =
                    inet::Coord(1, 0, 0));
    /** @brief gets the current target which the satellite is pointing to */
    virtual void getCurrentTarget(AttitudeTarget &currentTarget);
    /** @brief Returns the power consumption in the range [0, +infinity) */
    virtual inet::power::W getPowerConsumption() const;
    /** @brief Returns the energy source that provides energy for this energy consumer.*/
    virtual inet::power::IEnergySource* getEnergySource() const;
};

}  // namespace estnet

#endif
