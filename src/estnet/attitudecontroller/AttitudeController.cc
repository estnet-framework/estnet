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

#include "AttitudeController.h"

#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <cmath>
#include <numeric>

#include <omnetpp.h>

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/common/time/GlobalJulianDate.h"
#include "estnet/mobility/satellite/common/QuaternionHelpers.h"

namespace estnet {

Define_Module(AttitudeController);

omnetpp::simsignal_t AttitudeController::accA = registerSignal("accA");
omnetpp::simsignal_t AttitudeController::accB = registerSignal("accB");
omnetpp::simsignal_t AttitudeController::accG = registerSignal("accG");
omnetpp::simsignal_t AttitudeController::pointingError = registerSignal(
        "pointingError");

void AttitudeController::initialize(int stage) {
    if (stage == inet::INITSTAGE_LOCAL) {
        // this->maxAngularAcceleration = this->par("angularAcceleration");

        this->_lastUpdateTime =
                GlobalJulianDate::getInstance().currentSimTime();
        this->_lastUpdateTime.addSec(-std::numeric_limits<double>::max());

        this->_maxPower = inet::power::W(
                this->par("maxPower").doubleValueInUnit("W"));
        this->_idlePower = inet::power::W(
                this->par("idlePowerConsumption").doubleValueInUnit("W"));

        const char *energySourcePath = par("pathToEnergySource");
        if (getModuleByPath(energySourcePath) != nullptr)
            this->_energySource = omnetpp::check_and_cast<SimpleEpBattery*>(
                    getModuleByPath(energySourcePath));
        else
            EV_WARN << "Energy Source Module not found or not in use!"
                           << std::endl;

        std::stringstream ss(this->par("target").stdstringValue());
        std::string segment;
        std::vector<std::string> seglist;
        while (std::getline(ss, segment, ',')) {
            seglist.push_back(segment);
        }
        if (seglist.size() == 3) {
            this->target = *new AttitudeTarget(
                    inet::Coord(std::stod(seglist[0]), std::stod(seglist[1]),
                            std::stod(seglist[2])));
        } else {
            this->target = *new AttitudeTarget(
                    this->par("target").stdstringValue());
        }

        this->targetUpdateTime = this->par("targetUpdateTime");

        this->recordingUpdateTime = this->par("recordingUpdateTime");

        std::stringstream ss1(this->par("pointingAxis").stdstringValue());
        std::string segment1;
        std::vector<std::string> seglist1;
        while (std::getline(ss1, segment1, ',')) {
            seglist1.push_back(segment1);
        }
        this->pointingAxis = inet::Coord(std::stod(seglist1[0]),
                std::stod(seglist1[1]), std::stod(seglist1[2]));

        std::stringstream ss2(
                this->par("angularAcceleration").stdstringValue());
        std::string segment2;
        std::vector<std::string> seglist2;
        while (std::getline(ss2, segment2, ',')) {
            seglist2.push_back(segment2);
        }
        this->maxAngularAcceleration[0] = std::stod(seglist2[0]);
        if (seglist2.size() == 1) {
            this->maxAngularAcceleration[1] = std::stod(seglist2[0]);
            this->maxAngularAcceleration[2] = std::stod(seglist2[0]);
        } else {
            this->maxAngularAcceleration[1] = std::stod(seglist2[1]);
            this->maxAngularAcceleration[2] = std::stod(seglist2[2]);
        }

    } else if (stage == 8) {
        //register as consumer to battery
        if (_energySource != nullptr)
            _energySource->addEnergyConsumer(this);
    } else if (stage == inet::INITSTAGE_LAST) {
        this->attitudeControllerUpdate = new omnetpp::cMessage(
                "attitudeControllerUpdate");
        updateAttitude();
        if (this->recordingUpdateTime != 0) {
            this->recordingUpdate = new omnetpp::cMessage("recordingUpdate");
            this->scheduleAt(omnetpp::simTime() + this->recordingUpdateTime,
                    this->recordingUpdate);
        }
    }
}

void AttitudeController::handleMessage(omnetpp::cMessage *msg) {
    if (msg == this->attitudeControllerUpdate) {
        this->updateAttitude();
    } else if (msg == this->recordingUpdate) {
        this->recordePointingError();
    }
}

SatMobility* AttitudeController::getMobility() const {
    return dynamic_cast<SatMobility*>(this->getParentModule()->getSubmodule(
            "networkHost")->getSubmodule("mobility"));
}

void AttitudeController::updateAttitude() {
    this->calcAttitudeChange();
    if (this->attitudeControllerUpdate->isScheduled())
        cancelEvent(this->attitudeControllerUpdate);
    this->scheduleAt(omnetpp::simTime() + this->targetUpdateTime,
            this->attitudeControllerUpdate);
}

void AttitudeController::changeTarget(AttitudeTarget &newTarget,
        double targetUpdateTime, inet::Coord pointigAxis) {
    Enter_Method
    ("changeTarget");
    this->target = newTarget;
    this->targetUpdateTime = targetUpdateTime;
    this->pointingAxis = pointigAxis;
    this->_lastUpdateTime = GlobalJulianDate::getInstance().currentSimTime();
    this->_lastUpdateTime.addSec(-std::numeric_limits<double>::max());
    EV_INFO << "Updatet Attitude Target to " << this->target << omnetpp::endl;
    this->updateAttitude();
}

void AttitudeController::getCurrentTarget(AttitudeTarget &currentTarget) {
    currentTarget = this->target;
}

void AttitudeController::calcAttitudeChange() {
    auto mobility = getMobility();
    // calculating time difference since last update
    cJulian currentTime = GlobalJulianDate::getInstance().currentSimTime();
    double timeDifference = currentTime.spanSec(this->_lastUpdateTime);

    if (!this->target.isNil()) {
        if (mobility == nullptr) {
            throw omnetpp::cRuntimeError(
                    "The attitude controller expects the satellite to have a SatMobility as mobility module. If you don't need the attitude controller set the target to \"NIL\"");
        }

        if (timeDifference > 0) {
            // update lastUpdateTime
            this->_lastUpdateTime =
                    GlobalJulianDate::getInstance().currentSimTime();

            // set targeted time
            cJulian targetTime = currentTime;
            targetTime.addSec(this->targetUpdateTime);
            double targetSimTime = omnetpp::simTime().dbl()
                    + this->targetUpdateTime;

            // get orientation and position at currentTime + targetUpdateTime
            // and calculate the pointing direction at this time point
            double consumedEnergy = 0;
            inet::Coord ourFuturePos, targetFuturePos, pointingDirection;
            ourFuturePos = mobility->getPositionAtTime(targetSimTime);
            target.getTargetCoordAtTime(targetFuturePos, targetSimTime);
            pointingDirection = targetFuturePos - ourFuturePos;

            // calculate new orientation and orientation change to just propagated orientation
            cQuaternion newOrientation = cQuaternion::rotationFromTo(
                    this->pointingAxis, pointingDirection);
            cQuaternion newOrientationCon = newOrientation;
            newOrientationCon.conjugate();

            // calculate the angular velocity w to rotate from the current orientation
            // to the desired new orientation
            cQuaternion w;
            inet::Coord axis;
            double angle;
            cQuaternion q = mobility->getCurrentAngularPosition()
                    * newOrientationCon;
            q.normalize();
            // get the angle and axis of this rotation
            angle = 2 * std::acos(q.s);
            if (!std::isnan(angle) && angle != 0.0) {
                // invert angle if it is bigger than pi to rotate the other way around, because it is shorter
                if (angle > M_PI) {
                    angle -= 2 * M_PI;
                } else if (angle < -M_PI) {
                    angle += 2 * M_PI;
                }
                axis = q.getV() / q.getV().length();
                axis.normalize();

                axis *= -angle / this->targetUpdateTime;
                w.setV(axis);
            } else {
                w.setV(inet::Coord(0, 0, 0));
            }
            w.setS(0);
            cQuaternion currentAngularVelocity =
                    mobility->getCurrentAngularVelocity();
            cQuaternion dw = w - currentAngularVelocity;
            cQuaternion neededAngularAcceleration = dw / this->targetUpdateTime;

            /*
             * Checking if the neededAcceleration is greater then the maximum possible acceleration.
             * If this is the case we set the acceleration to the maximum possible and adjust the velocity and orientation.
             *
             * Note: angular velocity and acceleration is saved in a quaternion where the scalar part is 0 and the vector part
             * represents the angular velocity/acceleration around the X/Y/Z axis. Therefore the loop starts with 1 and goes to 4,
             * iterating only over the vector part.
             */
            for (int i = 1; i < 4; i++) {
                if (abs(getQuaternionValueAt(neededAngularAcceleration, i))
                        > this->maxAngularAcceleration[i - 1]) {
                    EV_WARN
                                   << "ATTITUDE CONTROLLER IS USING MAXIMUM ACCELERATION"
                                   << omnetpp::endl;
                    int sign;
                    if (getQuaternionValueAt(neededAngularAcceleration, i)
                            >= 0) {
                        sign = 1;
                    } else {
                        sign = -1;
                    }
                    setQuaternionValueAt(neededAngularAcceleration, i,
                            this->maxAngularAcceleration[i - 1] * sign);
                    setQuaternionValueAt(w, i,
                            getQuaternionValueAt(currentAngularVelocity, i)
                                    + sign * this->maxAngularAcceleration[i - 1]
                                            * this->targetUpdateTime);
                }
            }

            // calculating consumed energy
            cQuaternion powerPerAxis;
            for (int i = 1; i < 4; i++) {
                double energy = abs(
                        this->_maxPower.get()
                                / this->maxAngularAcceleration[i - 1]
                                * getQuaternionValueAt(
                                        neededAngularAcceleration, i));
                setQuaternionValueAt(powerPerAxis, i, energy);
                consumedEnergy += energy;
            }

            // setting future attitude
            cJulian newTime = currentTime;
            newTime.addSec(this->targetUpdateTime);

            mobility->externalAttitudeUpdateQuaternion(
                    mobility->getCurrentAngularPosition(), w,
                    mobility->getCurrentAngularAcceleration(), currentTime);

            this->_powerConsumption = inet::power::W(consumedEnergy)
                    + this->_idlePower;
            emit(inet::power::IEpEnergySource::powerConsumptionChangedSignal,
                    _powerConsumption.get());
            this->emit(accA, neededAngularAcceleration.getV().getX());
            this->emit(accB, neededAngularAcceleration.getV().getY());
            this->emit(accG, neededAngularAcceleration.getV().getZ());
            this->emit(pointingError, angle / M_PI * 180);
        }
    }
    // the satellite is not tracking anything, so only idle power is used
    else {
        if (timeDifference > 0) {
            this->_powerConsumption = this->_idlePower;
            emit(inet::power::IEpEnergySource::powerConsumptionChangedSignal,
                    _powerConsumption.get());
        }
    }
}

void AttitudeController::recordePointingError() {
    if (!this->target.isNil()) {
        double lastUpdateSimTime =
                GlobalJulianDate::getInstance().julianDate2SimTime(
                        _lastUpdateTime);

        EV << "Recording0: " << lastUpdateSimTime << omnetpp::endl;
        double targetSimTime = lastUpdateSimTime + this->targetUpdateTime;
        EV << "Recording1: " << targetSimTime << omnetpp::endl;

        auto mobility = getMobility();
        inet::Coord ourFuturePos, targetFuturePos, pointingDirection;
        ourFuturePos = mobility->getPositionAtTime(targetSimTime);
        target.getTargetCoordAtTime(targetFuturePos, targetSimTime);
        pointingDirection = targetFuturePos - ourFuturePos;

        // calculate new orientation and orientation change to just propagated orientation
        cQuaternion newOrientation = cQuaternion::rotationFromTo(
                this->pointingAxis, pointingDirection);
        cQuaternion newOrientationCon = newOrientation;
        newOrientationCon.conjugate();

        double angle;
        cQuaternion q = mobility->getCurrentAngularPosition()
                * newOrientationCon;
        q.normalize();
        // get the angle and axis of this rotation
        angle = 2 * std::acos(q.s);
        if (!std::isnan(angle)) {
            // invert angle if it is bigger than pi to rotate the other way around, because it is shorter
            if (angle > M_PI) {
                angle -= 2 * M_PI;
            } else if (angle < -M_PI) {
                angle += 2 * M_PI;
            }
        } else {
            angle = 0;
        }
        this->emit(pointingError, angle / M_PI * 180);
    }
    this->scheduleAt(omnetpp::simTime() + this->recordingUpdateTime,
            this->recordingUpdate);
}

inet::power::W AttitudeController::getPowerConsumption() const {
    return this->_powerConsumption;
}

inet::power::IEnergySource* AttitudeController::getEnergySource() const {
    return this->_energySource;
}

AttitudeController::~AttitudeController() {
}

void AttitudeController::finish() {
    this->cancelAndDelete(this->attitudeControllerUpdate);
    omnetpp::cSimpleModule::finish();
}

}  // namespace estnet
