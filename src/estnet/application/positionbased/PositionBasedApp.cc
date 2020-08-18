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

#include "PositionBasedApp.h"

#include <cmath>
#include <string>
#include <regex>

#include <inet/common/geometry/common/Coord.h>
#include <inet/common/INETMath.h>

#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/environment/earthmodel/EarthModelFactory.h"
#include "estnet/global_config.h"
#include "common/AISDataLoader.h"
#include "common/GeoCoordinateDataHandler.h"
#include "common/MemorizedDataHandler.h"

namespace estnet {

Define_Module(PositionBasedApp);

void PositionBasedApp::initialize(int stage) {
    BasicApp::initialize(0);

    this->earthModel = EarthModelFactory::get(
            EarthModelFactory::EarthModels::WGS84);

    omnetpp::cModule *mobilityModule =
            this->getParentModule()->getParentModule()->getSubmodule(
                    "mobility");
    try {
        satMobility = dynamic_cast<SatMobility*>(mobilityModule);
    } catch (const omnetpp::cRuntimeError &e) {
        satMobility = nullptr;
    }

    // loads data source
    std::string s = this->par("dataSource").stringValue();
    std::smatch m;
    std::regex r_coordinats("^(\\-?\\d+(\\.\\d+)?),\\s(\\-?\\d+(\\.\\d+)?)");
    if (s.compare("AIS") == 0) {
        pd = new AISDataLoader(this->par("dataPath").stdstringValue());
    } else if (s.compare("Memorized") == 0) {
        pd = MemorizedDataHandler::getInstance(satMobility, earthModel,
                this->par("dataPath").stdstringValue());
    } else if (regex_search(s, m, r_coordinats)) {
        pd = new GeoCoordinateDataHandler(s);
    } else {
        error("Invalid data source for position-based app %s",
                this->par("dataSource").stringValue());
    }

    this->_startTime = par("startTime").doubleValue();
    this->_stopTime = par("stopTime").doubleValue();
    this->_scheduleMsgEmpty = new omnetpp::cMessage("appScheduleEmpty");

    //schedule packet sending
    omnetpp::simtime_t nextTime;
    if (this->par("sending").boolValue() && this->nextPacketTime(nextTime)) {
        this->scheduleAt(nextTime, this->_scheduleMsgEmpty);
    }
}

PositionBasedApp::~PositionBasedApp() {
    this->cancelAndDelete(this->_scheduleMsgEmpty);
}

void PositionBasedApp::handleMessage(omnetpp::cMessage *msg) {
    if (msg == this->_scheduleMsgEmpty) {
        this->scheduleNextPacket();
    } else {
        BasicApp::handleMessage(msg);
    }
}

void PositionBasedApp::scheduleNextPacket(omnetpp::simtime_t nextTime) {
    // figure out when to send the next packet
    double multiplier = this->getPositionMultiplier();
    if (multiplier > 1e-300) {
        if (this->nextPacketTime(nextTime)) {
            double sendInterval = this->par("basicDataInterval").doubleValue()
                    / multiplier;
            if (omnetpp::simTime() == 0.0) {
                nextTime = this->_startTime + sendInterval;
            } else {
                nextTime = omnetpp::simTime() + sendInterval;
            }
            BasicApp::scheduleNextPacket(nextTime);
        }
    } else {
        if (this->nextPacketTime(nextTime)) {
            this->scheduleAt(nextTime, this->_scheduleMsgEmpty);
        }
    }
}

bool PositionBasedApp::nextPacketTime(omnetpp::simtime_t &t) {
    double sendInterval = this->par("idleInterval").doubleValue();
    if (omnetpp::simTime() == 0.0) {
        t = this->_startTime + sendInterval;
    } else {
        t = omnetpp::simTime() + sendInterval;
    }
    if (sendInterval > 0
            && (this->_stopTime < SIMTIME_ZERO || t < this->_stopTime)) {
        return true;
    }
    return false;
}

double PositionBasedApp::getPositionMultiplier() {
    if (satMobility != nullptr) {
        inet::Coord coord = this->satMobility->getCurrentPosition();
        inet::deg latitude, longitude;
        inet::m altitude;
        double multiplier;
        this->earthModel->convertECIToLatLongHeight(
                GlobalJulianDate::getInstance().currentSimTime(), coord,
                latitude, longitude, altitude);
        // calculate altitude manually, because convertECITo... returns altitude
        // above ground and we need the altitude above the center of the earth
        altitude = inetu::m(
                std::sqrt(
                        std::pow(coord.x, 2) + std::pow(coord.y, 2)
                                + std::pow(coord.z, 2)));

        if (this->par("maxFOV").boolValue()) {
            beamWidth = inet::math::rad2deg(
                    2.0 * std::asin(EARTH_AVG_R / altitude.get()));
        } else {
            beamWidth = this->par("beamWidth").doubleValue();
        }

        pd->getDataForCone(latitude.get(), longitude.get(), altitude.get(),
                beamWidth, multiplier);

        // normal distribution with multiplier as mean
        if (this->par("useNormalOfMultiplier").boolValue()) {
            multiplier = this->getParentModule()->normal(multiplier, 1);
        }
        return multiplier;
    }
    return -1;
}

}  // namespace estnet
