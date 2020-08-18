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

#include "EnergyModule.h"

#include <inet/common/geometry/object/LineSegment.h>
#include <inet/common/geometry/base/ShapeBase.h>
#include <inet/common/geometry/shape/Sphere.h>
#include <inet/environment/common/PhysicalEnvironment.h>

#include "estnet/node/satellite/Satellite.h"

using namespace estnet;

Define_Module(EnergyModule);

void EnergyModule::initialize(int stage) {
    if (stage == 0) {
        //initialize parameter
        this->_supplyConsumerDirectly = this->par("supplyConsumerDirectly");

        // calculate Sun Position, setting sun depending on the angle set in ini-file into the x-z-plane of ECI
        double sunOrientation = this->par("sunAngle");
        this->_sunPosition = inet::Coord();
        if (std::abs(sunOrientation) > 23.5) {
            EV_ERROR << " invalid sun angle"
                            << "should be between -23.5 to 23.5 deg";
        }
        sunOrientation = sunOrientation * M_PI / 180;
        double R_SUN = 150000000000.0; //distance sun <-> earth, in m
        this->_sunPosition.x = R_SUN * cos(sunOrientation);
        this->_sunPosition.y = 0.0;
        this->_sunPosition.z = R_SUN * sin(sunOrientation);

        //grab mobility of this node
        this->_mobility = omnetpp::check_and_cast<Satellite*>(
                this->getParentModule())->getMobility();
    }
}

inet::Coord EnergyModule::getSunPosition() {
    return this->_sunPosition;
}

bool EnergyModule::isInEclipse() {
    // calculate sun sat line and check if it intersects with earth sphere
    inet::LineSegment line = inet::LineSegment(
            this->_mobility->getCurrentPosition() / 1000000 /*scaling by
             1 million meter to prevent overflow*/, this->_sunPosition / 1000000);
    inet::physicalenvironment::PhysicalEnvironment *pEnv =
            omnetpp::check_and_cast<
                    inet::physicalenvironment::PhysicalEnvironment*>(
                    this->getSystemModule()->getSubmodule(
                            "physicalEnvironment"));
    const inet::ShapeBase *shape = pEnv->getObject(0)->getShape();
    inet::Sphere *earth = const_cast<inet::Sphere*>(omnetpp::check_and_cast<
            const inet::Sphere*>(shape));
    earth->setRadius(earth->getRadius() / 1000000);
    EV_DEBUG << "Radius: " << earth->getRadius() << "Line: ("
                    << line.getPoint1().x << " , " << line.getPoint1().y
                    << " , " << line.getPoint1().z << " ) - ( "
                    << line.getPoint2().x << " , " << line.getPoint2().y
                    << " , " << line.getPoint2().z << " )" << std::endl;
    inet::Coord c1, c2, c3, c4;
    bool rtn = earth->computeIntersection(line, c1, c2, c3, c4);
    if (rtn)
        EV_DEBUG << "In Eclipse!" << std::endl;
    earth->setRadius(earth->getRadius() * 1000000);
    return rtn;
}
