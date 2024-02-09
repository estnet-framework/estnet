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

#include "TargetTrackingConsumer.h"

using namespace estnet;

Define_Module(TargetTrackingConsumer);

TargetTrackingConsumer::~TargetTrackingConsumer() {
    cancelAndDelete(this->_changeStatus);

}

void TargetTrackingConsumer::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_PHYSICAL_OBJECT_CACHE) {
        //get parameters from ini file
        this->_consumption = W(this->par("powerConsumption").doubleValue());
        this->_consumption = this->_consumption
                / this->par("efficiency").doubleValue();
        this->_offConsumption = W(
                this->par("offPowerConsumption").doubleValue());
        this->_offConsumption = this->_offConsumption
                / this->par("efficiency").doubleValue();
        _energyConsumer = check_and_cast<ConsumerModuleBase*>(
                getParentModule());
        this->_latitude = inet::deg(
                this->par("latitude").doubleValueInUnit("deg"));
        this->_longitude = inet::deg(
                this->par("longitude").doubleValueInUnit("deg"));
        this->_minElevation = inet::deg(
                this->par("minElevation").doubleValueInUnit("deg"));
        //publish power consumptions once, starting in state off
        this->_on = false;

        this->_earthModel = EarthModelFactory::get(
                EarthModelFactory::EarthModels::WGS84);
        //starting timer for turning off
        this->_changeStatus = new cMessage("switchStatus");
        scheduleAt(SimTime(0, SIMTIME_S), this->_changeStatus);
    } else if (stage == 6) {
        // power initializing
        _energyConsumer->addStateHandler(this);
        this->updatePowerConsumption();

        //save current target, in order to switch back to it after tracking
        AttitudeController *att =
                check_and_cast<AttitudeController*>(
                        (this->getModuleByPath(
                                par("pathToEnergyModule").stringValue()))->getParentModule()->getModuleByPath(
                                ".attitudeController"));
        att->getCurrentTarget(_lastTarget);

    }

}

void TargetTrackingConsumer::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        //get satellite's position
        Satellite *sat;
        try {
            auto energyModule = this->getModuleByPath(
                    par("pathToEnergyModule").stringValue());
            sat = check_and_cast<Satellite*>(energyModule->getParentModule());
        } catch (std::exception &e) {
            throw omnetpp::cRuntimeError(
                    "Target Tracker has not found Energy Module:"
                            " Make sure you set the correct path!");
        }
        inet::Coord coordinates = sat->getMobility()->getCurrentPosition();

        //calculate elevation to target point
        inetu::deg elevation = this->_earthModel->calculateElevation(
                coordinates, _latitude, _longitude, inetu::m(0));

        //if elevation is negative, schedule in 60s, otherwise schedule in 1s
        if (elevation >= (this->_minElevation - inetu::deg(5))) {
            scheduleAt(simTime() + SimTime(1, SIMTIME_S), msg);
        } else {
            scheduleAt(simTime() + SimTime(60, SIMTIME_S), msg);
        }
        //change state dependent on the elevation
        if (elevation >= this->_minElevation) {
            if (!this->_on) {
                this->_on = true;
                updatePowerConsumption();
                updateTracking();
            }
        } else {
            if (this->_on) {
                this->_on = false;
                updatePowerConsumption();
                updateTracking();
            }
        }

    }
}

void TargetTrackingConsumer::updatePowerConsumption() {

    this->_powerConsumption =
            this->_on ? this->_consumption : this->_offConsumption;
    EV_DEBUG << "Publishing a param range dc consume of "
                    << this->_powerConsumption << std::endl;
    this->_energyConsumer->powerConsumptionChanged();

}

void TargetTrackingConsumer::updateTracking() {
    AttitudeController *att =
            check_and_cast<AttitudeController*>(
                    (this->getModuleByPath(
                            par("pathToEnergyModule").stringValue()))->getParentModule()->getModuleByPath(
                            ".attitudeController"));
    if (this->_on) {
        //save current target, in order to switch back to it after tracking
        att->getCurrentTarget(_lastTarget);

        // start target tracking
        inet::Coord point = inet::Coord();
        this->_earthModel->convertLatLongHeightToECI(
                GlobalJulianDate::getInstance().currentSimTime(),
                this->_latitude, this->_longitude, inetu::m(0), point);
        _target = AttitudeTarget(point);
        att->changeTarget(_target);
    } else {
        att->changeTarget(_lastTarget);
    }
}

