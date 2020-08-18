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

#include "INodeTracking.h"

#include "../../groundstation/GroundStation.h"

namespace estnet {

void INodeTracking::initialize() {
    // read the parameter
    this->_updateInterval = inetu::s(
            par("updateInterval").doubleValueInUnit("s"));
    this->_lastUpdate = GlobalJulianDate::getInstance().currentSimTime();
    this->_isTracking = false;
    this->_enabled = par("enable");
}

void INodeTracking::start() {
    this->_isTracking = this->_enabled;
}
void INodeTracking::stop() {
    turnToDefault();
    this->_isTracking = false;
}

bool INodeTracking::isTracking() const {
    return _isTracking;
}

void INodeTracking::updateAttitude() {
    if (!_isTracking) {
        turnToDefault();
    } else if (GlobalJulianDate::getInstance().currentSimTime().spanSec(
            _lastUpdate) > _updateInterval.get()) {
        auto groundstation = check_and_cast<GroundStation*>(
                this->getParentModule());
        auto mobility = groundstation->getMobility();
        auto attitude = this->getNewOrientation();
        mobility->setCurrentAngularPosition(attitude);
        this->_lastUpdate = GlobalJulianDate::getInstance().currentSimTime();
    }
}

inet::Quaternion INodeTracking::turnToDefault() {
    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    StaticTerrestrialMobility *m = groundstation->getMobility();
    inet::Coord ourPositionECI = m->getCurrentPosition();
    //point to zenith
    auto defaultOrientation = inet::Quaternion::rotationFromTo(
            inet::Coord(1, 0, 0), ourPositionECI);
    m->setCurrentAngularPosition(defaultOrientation);
    return defaultOrientation;
}

} //namespace
