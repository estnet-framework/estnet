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

#include "Satellite.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/global_config.h"

namespace estnet {

Define_Module(Satellite);

int Satellite::numInitStages() const {
    return 2;
}

void Satellite::initialize(int stage) {
    NodeBase::initialize(stage);
    if (stage == 1) {
        NodeRegistry::getInstance()->addSatellite(this);
        this->alignmentCheckEnable = par("alignmentCheckEnable");
        this->contactTrackingEnable = par("contactTrackingEnable");
        this->contactTrackingUpdateTime =
                par("contactTrackingUpdateTime").doubleValue();
        this->attCon = dynamic_cast<AttitudeController*>(this->getSubmodule(
                "attitudeController"));
    }

}

bool Satellite::getAlignmentCheckEnable() {
    return this->alignmentCheckEnable;
}

IExtendedMobility* Satellite::getMobility() const {
    return dynamic_cast<IExtendedMobility*>(this->getSubmodule("networkHost")->getSubmodule(
            "mobility"));
}

void Satellite::addContactTo(unsigned int other_node_no) {
    if (contactTrackingEnable) {
        //Track other_node_no
        estnet::AttitudeTarget target = *new estnet::AttitudeTarget(
                other_node_no);
        attCon->changeTarget(target, contactTrackingUpdateTime,
                inet::Coord::X_AXIS);
    }

    //TODO
    if (dynamic_cast<Satellite*>(NodeRegistry::getInstance()->getNode(
            other_node_no)) != nullptr) {
        Satellite *targetSat =
                dynamic_cast<Satellite*>(NodeRegistry::getInstance()->getNode(
                        other_node_no));
        if (targetSat->getAlignmentCheckEnable()) {
            estnet::AttitudeTarget targetSatTarget =
                    *new estnet::AttitudeTarget(this->_nodeNo);
            targetSat->attCon->changeTarget(targetSatTarget,
                    targetSat->contactTrackingUpdateTime, inet::Coord::X_AXIS);
        }
    }

    NodeBase::addContactTo(other_node_no);
}

void Satellite::removeContactTo(unsigned int other_node_no) {
    if (contactTrackingEnable) {
        //Track other_node_no
        estnet::AttitudeTarget target = *new estnet::AttitudeTarget("SUN");
        attCon->changeTarget(target, contactTrackingUpdateTime,
                inet::Coord::X_AXIS);
    }

    //TODO
    if (dynamic_cast<Satellite*>(NodeRegistry::getInstance()->getNode(
            other_node_no)) != nullptr) {
        Satellite *targetSat =
                dynamic_cast<Satellite*>(NodeRegistry::getInstance()->getNode(
                        other_node_no));
        if (targetSat->getAlignmentCheckEnable()) {
            estnet::AttitudeTarget targetSatTarget =
                    *new estnet::AttitudeTarget("SUN");
            targetSat->attCon->changeTarget(targetSatTarget,
                    targetSat->contactTrackingUpdateTime, inet::Coord::X_AXIS);
        }
    }

    NodeBase::removeContactTo(other_node_no);
}

}  // namespace estnet
