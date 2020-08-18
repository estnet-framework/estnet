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

#include "OsgNode.h"
#include "estnet/node/base/NodeBase.h"
#include "common/OsgUtils.h"
#include "estnet/mobility/satellite/common/QuaternionHelpers.h"

#include <iomanip>
#include <sstream>
#include <string.h>

#ifdef WITH_OSG
#include <omnetpp/osgutil.h>

#include <osg/Depth>
#include <osg/Image>
#include <osg/LineWidth>
#include <osg/Node>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/Texture>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Geometry>

#include <osgEarth/MapNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/Style>
#include <osgEarthUtil/LineOfSight>
#include <osgEarthUtil/LinearLineOfSight>
#endif

#include "OsgEarthScene.h"

namespace estnet {

Define_Module(OsgNode);

void OsgNode::initialize(int stage) {
    switch (stage) {
    case 0: {
        /// read ned-file parameters
        timeStep = par("timeStep");
        modelURL = par("modelURL").stringValue();
        modelScale = par("modelScale");
        labelColor = par("labelColor").stringValue();
        orbitColor = par("orbitColor").stringValue();
        orbitResolution = par("orbitResolution");
        orbitUpdateFreq = par("orbitUpdateFreq");
        orbitNextUpdate = orbitUpdateFreq;
        satConeColor = par("satConeColor").stringValue();
        satConeScaling = par("satConeScaling").doubleValue();
        coordBaseColorX = par("coordBaseColorX").stringValue();
        coordBaseColorY = par("coordBaseColorY").stringValue();
        coordBaseColorZ = par("coordBaseColorZ").stringValue();
        coordBaseLength = par("coordBaseLength").doubleValue();
        coordBaseWidth = par("coordBaseWidth").doubleValue();
        break;
    }
        // after OSG initialization
    case 1: {
        /// path to the mobility-module
        omnetpp::cModule *mobilityModule =
                this->getParentModule()->getSubmodule("networkHost")->getSubmodule(
                        "mobility");
        // check if the module is satMobility or staticTerrestrielMobility
        try {
            satMobility = dynamic_cast<SatMobility*>(mobilityModule);
        } catch (const omnetpp::cRuntimeError &e) {
            satMobility = nullptr;
        }
        if (satMobility == nullptr) {
            try {
                staticTerrestrialMobility =
                        dynamic_cast<StaticTerrestrialMobility*>(mobilityModule);
            } catch (const omnetpp::cRuntimeError &e) {
                staticTerrestrialMobility = nullptr;
            }
        }
        // if there is no compatible module, throw error
        if (satMobility == nullptr && staticTerrestrialMobility == nullptr) {
            throw omnetpp::cRuntimeError(
                    "No suitable mobility module found for OsgNode Module %s !\n",
                    this->getFullPath().c_str());
        }

#ifdef WITH_OSG
        // OSG-Node initialization
        auto scene = OsgEarthScene::getInstance()->getScene(); // get global OSG scene
        mapNode = osgEarth::MapNode::findMapNode(scene);

        geoTransform = new osgEarth::GeoTransform(); // build up the node representing this module

        auto modelNode = osgDB::readNodeFile(modelURL);

        modelNode->getOrCreateStateSet()->setAttributeAndModes(
                new osg::Program(),
                osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
        modelNode->getOrCreateStateSet()->setMode(
        GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

        // scale and rotate the model if necessary
        auto pat = new osg::PositionAttitudeTransform();
        pat->setScale(osg::Vec3d(modelScale, modelScale, modelScale));

        auto objectNode = new omnetpp::cObjectOsgNode(this->getParentModule()); //->getParentModule()??
        objectNode->addChild(modelNode);
        geoTransform->addChild(pat);

        // if the model isn't rotated the same way as inet uses it by default
        _modelMatrixTransform = new osg::MatrixTransform();
        pat->addChild(_modelMatrixTransform);

        _modelMatrixTransform->addChild(objectNode);

        // create icon label
        if (!labelColor.empty()) {
            osgEarth::Symbology::Style labelStyle;
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->alignment() =
                    osgEarth::Symbology::TextSymbol::ALIGN_CENTER_TOP;
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->declutter() =
                    true;
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->pixelOffset() =
                    osg::Vec2s(0, 50);
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->fill()->color() =
                    osgEarth::Color(labelColor);
            // Construct Label-String
            std::ostringstream os;
            os << this->getParentModule()->par("label").stringValue() << "\n"
                    << this->getParentModule()->getFullName();
            std::string completeLabel = os.str();
            geoTransform->addChild(
                    new osgEarth::Annotation::LabelNode(completeLabel,
                            labelStyle));
        }

        scene->asGroup()->addChild(geoTransform); // add the locator node to the scene

        // creating orbit circle

        // only print a circle if there is a color set and there is satMobility
        // mobility module
        if (!orbitColor.empty() && satMobility != nullptr) {
            _orbitGeode = new osg::Geode;
            this->drawOrbit(0);
            scene->asGroup()->addChild(_orbitGeode);
        }

        // draw satellite communication cone
        this->drawSatelliteCone();

        this->_drawables = new osg::Geode();
        _modelMatrixTransform->addChild(this->_drawables);
#endif

        // Initialization is finished.
        // Schedule first timer as message
        auto timer = new omnetpp::cMessage("positionUpdate");
        scheduleAt(0, timer);
        break;
    }
    }
}

void OsgNode::handleMessage(omnetpp::cMessage *msg) {
    // Schedule next movement to keep position data updated
    scheduleAt(omnetpp::simTime() + timeStep, msg);

#ifdef WITH_OSG
    // update orbit
    // with sgp4 the orbits starts shifting, so we'll need to update it
    if (!orbitColor.empty() && satMobility != nullptr) {
        double time = getSimulation()->getSimTime().inUnit(omnetpp::SIMTIME_S);
        if (time >= orbitNextUpdate) {
            EV_INFO << "updating orbit, time " << time << " orbitNextUpdate "
                           << orbitNextUpdate << omnetpp::endl;
            this->_orbitGeode->removeDrawables(0,
                    this->_orbitGeode->getNumDrawables());
            drawOrbit(time);
            orbitNextUpdate += orbitUpdateFreq;
        }
    }
#endif
}

void OsgNode::refreshDisplay() const {
#ifdef WITH_OSG
    osg::Vec3d pos, att;
    cJulian time = GlobalJulianDate::getInstance().currentSimTime();
    // get position from satmobility-module with respect to simulation time
    inet::Coord tempPos;
    M4x4d tempAtt;
    if (satMobility != nullptr) {
        tempPos = satMobility->getCurrentPosition();
        tempAtt = satMobility->getCurrentAngularPositionRelativeToENU(time,
                tempPos);
        /*double attitudeMatPtr[3][3];
         quaternionToMatrix(satMobility->getCurrentAngularPosition(), attitudeMatPtr);
         tempAtt << attitudeMatPtr[0][0], attitudeMatPtr[0][1], attitudeMatPtr[0][2], 0.0,
         attitudeMatPtr[1][0], attitudeMatPtr[1][1], attitudeMatPtr[1][2], 0.0,
         attitudeMatPtr[2][0], attitudeMatPtr[2][1], attitudeMatPtr[2][2], 0.0,
         0.0, 0.0, 0.0, 1.0;*/
    } else if (staticTerrestrialMobility != nullptr) {
        tempPos = staticTerrestrialMobility->getCurrentPosition();
        tempAtt =
                staticTerrestrialMobility->getCurrentAngularPositionRelativeToENU(
                        time, tempPos);
    } else {
        throw omnetpp::cRuntimeError(
                "No suitable mobility module found for OsgNode Module %s !\n",
                this->getFullPath().c_str());
    }

    pos[0] = tempPos.x;
    pos[1] = tempPos.y;
    pos[2] = tempPos.z;
    osg::Matrixd tempAttOsg(tempAtt(0, 0), tempAtt(0, 1), tempAtt(0, 2),
            tempAtt(0, 3), tempAtt(1, 0), tempAtt(1, 1), tempAtt(1, 2),
            tempAtt(1, 3), tempAtt(2, 0), tempAtt(2, 1), tempAtt(2, 2),
            tempAtt(2, 3), tempAtt(3, 0), tempAtt(3, 1), tempAtt(3, 2),
            tempAtt(3, 3));

    osg::Vec3d v;
    mapNode->getMap()->getSRS()->transformFromWorld(pos, v);

    // Update OSG 3D visualization
    geoTransform->setPosition(osgEarth::GeoPoint(mapNode->getMapSRS(), v));

    _modelMatrixTransform->setMatrix(tempAttOsg);

    // Also update the position on 2D canvas. Have to reach upper satellite
    // module
    this->getParentModule()->getDisplayString().setTagArg("p", 0,
            300 + pos.x() / 100000);
    this->getParentModule()->getDisplayString().setTagArg("p", 1,
            300 - pos.y() / 100000);

    // update coordinate system
    this->_drawables->removeDrawables(0, this->_drawables->getNumDrawables());
    drawCoordinateSystem(this->_drawables, this->coordBaseColorX,
            this->coordBaseColorY, this->coordBaseColorZ, this->coordBaseLength,
            this->coordBaseWidth);
#endif
}

void OsgNode::drawOrbit(double startTime) const {
#ifdef WITH_OSG
    osg::ref_ptr<osg::Geometry> orbitGeom = new osg::Geometry;
    osg::ref_ptr<osg::DrawArrays> drawArrayLines = new osg::DrawArrays(
            osg::PrimitiveSet::LINE_STRIP);
    osg::ref_ptr<osg::Vec3Array> vertexData = new osg::Vec3Array;

    orbitGeom->addPrimitiveSet(drawArrayLines);
    auto stateSet = orbitGeom->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setAttributeAndModes(new osg::LineWidth(1.5),
            osg::StateAttribute::ON);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    auto depth = new osg::Depth;
    depth->setWriteMask(false);
    stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

    double T = satMobility->getOrbitalPeriod();
    double endTime = startTime + T;
    double time = startTime;
    inet::Coord tempPos;

    while (time <= endTime) {
        tempPos = satMobility->getPositionAtTime(time);
        auto vec = new osg::Vec3(tempPos.x, tempPos.y, tempPos.z);
        vertexData->push_back(*vec);
        time += T / orbitResolution;
    }

    orbitGeom->setVertexArray(vertexData);
    drawArrayLines->setFirst(0);
    drawArrayLines->setCount(vertexData->size());

    osg::ref_ptr<osg::Vec4Array> colorData = new osg::Vec4Array;
    colorData->push_back(osgEarth::Color(orbitColor));
    orbitGeom->setColorArray(colorData, osg::Array::BIND_OVERALL);
    this->_orbitGeode->addDrawable(orbitGeom.get());
#endif
}

void OsgNode::drawSatelliteCone() const {
#ifdef WITH_OSG
    if (!this->satConeColor.empty()) {    // && satMobility != nullptr) {
//        double earthRadius = EARTH_AVG_R / 1000.0;
//        double orbitRadius = satMobility->getOrbitalRadius();
//        // the angle between the center of the earth and the horizon as seen from
//        // the satellite, in radians
//        double alpha = std::asin(earthRadius / orbitRadius);
//        // the distance of the horizon from the satellite, in meters
//        double horizonDistance = std::sqrt(
//                orbitRadius * orbitRadius - earthRadius * earthRadius) * 1000;
//        double coneHeight = std::sin(alpha) * horizonDistance;
//        double coneRadius = std::cos(alpha) * horizonDistance;
//        // the offset is to position the tip to the satellite

        auto cone = new osg::Cone(osg::Vec3(3.5e6 * 0.75, 0, 0),
                2e5 * this->satConeScaling, 3.5e6);
        cone->setRotation(osg::Quat(0, -0.7071068, 0, 0.7071068));

        osg::ref_ptr<osg::Geode> coneGeode = new osg::Geode;
        auto coneDrawable = new osg::ShapeDrawable(cone);
        coneDrawable->setColor(osgEarth::Color(this->satConeColor));

        coneGeode->addDrawable(coneDrawable);
        coneGeode->getOrCreateStateSet()->setAttribute(
                new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
                        osg::PolygonMode::LINE));
        coneGeode->getOrCreateStateSet()->setRenderingHint(
                osg::StateSet::TRANSPARENT_BIN);
        auto depth = new osg::Depth;
        depth->setWriteMask(false);
        coneGeode->getOrCreateStateSet()->setAttributeAndModes(depth,
                osg::StateAttribute::ON);
        _modelMatrixTransform->addChild(coneGeode);
    }
#endif
}

}  // namespace estnet
