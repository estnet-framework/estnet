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

#ifdef WITH_OSG
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgEarth/Capabilities>
#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>
#include <osgEarthAnnotation/RectangleNode>
#include <osgEarth/Notify>

#include "common/OsgUtils.h"
#endif

#include "OsgEarthScene.h"

#include "estnet/global_config.h"

namespace estnet {

Define_Module(OsgEarthScene);

OsgEarthScene *OsgEarthScene::instance = nullptr;

OsgEarthScene::OsgEarthScene() {
    if (instance != nullptr) {
        throw omnetpp::cRuntimeError(
                "There can be only one OsgRenderer instance in the network");
    }
    instance = this;
}

OsgEarthScene::~OsgEarthScene() {
    instance = nullptr;
}

void OsgEarthScene::initialize() {
#ifdef WITH_OSG
    /*Suppresses all osgearth notifications with notify level warning or higher
     * since otherwise the console is unusable.*/
    osgEarth::setNotifyLevel(osg::FATAL);
    coordBaseColorX = par("coordBaseColorX").stringValue();
    coordBaseColorY = par("coordBaseColorY").stringValue();
    coordBaseColorZ = par("coordBaseColorZ").stringValue();
    coordBaseLength = par("coordBaseLength").doubleValue();
    coordBaseWidth = par("coordBaseWidth").doubleValue();

    auto sceneModelPath = par("sceneModel").stringValue();

    auto mapNode = dynamic_cast<osgEarth::MapNode*>(osgDB::readNodeFile(
            sceneModelPath));
    if (mapNode == nullptr) {
        throw omnetpp::cRuntimeError("Could not read scene file \"%s\"",
                sceneModelPath);
    }

    omnetpp::cOsgCanvas *builtinOsgCanvas = getParentModule()->getOsgCanvas();

    builtinOsgCanvas->setCameraManipulatorType(
            omnetpp::cOsgCanvas::CAM_TRACKBALL);

    // set up viewer
    builtinOsgCanvas->setViewerStyle(omnetpp::cOsgCanvas::STYLE_EARTH);
    builtinOsgCanvas->setClearColor(omnetpp::cOsgCanvas::Color("black"));
    builtinOsgCanvas->setZNear(100000);
    builtinOsgCanvas->setZFar(1000000000);

    earthRotator = new osg::PositionAttitudeTransform();
    earthRotator->addChild(mapNode);

    osg::PositionAttitudeTransform *eciRotator =
            new osg::PositionAttitudeTransform();

    scene = new osg::Group();
    scene->addChild(earthRotator);

    int year, month;
    double dayOfMonth;
    auto startTime = GlobalJulianDate::getInstance().simTime2JulianDate(0);
    startTime.getComponent(&year, &month, &dayOfMonth);
    int days = (int) dayOfMonth;
    dayOfMonth -= (double) days;
#if OSGEARTH_VERSION_GREATER_OR_EQUAL(2, 6, 0)
    skynode = osgEarth::Util::SkyNode::create(mapNode);
    osgEarth::DateTime date(year, month, days, dayOfMonth * 24.0);
    skynode->setDateTime(date);
    skynode->setMoonVisible(true);
    earthRotator->addChild(skynode);
    //scene->addChild(osgEarth::Util::SkyNode::create(mapNode));
#else
  skynode = new osgEarth::Util::SkyNode(mapNode->getMap());
  int year, month;
  double dayOfMonth;
  simStartTime.getComponent(&year, &month, &dayOfMonth);
  int days = (int)dayOfMonth;
  dayOfMonth -= (double)days;
  skynode->setDateTime(year, month, days, dayOfMonth*24);
  skynode->setMoonVisible(true);
  earthRotator->addChild(skynode);
#endif
    auto stateSet = scene->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    mapNode->getOrCreateStateSet()->setMode(GL_LIGHTING,
            osg::StateAttribute::ON);

    this->_drawables = new osg::Geode();
    drawCoordinateSystem(this->_drawables, this->coordBaseColorX,
            this->coordBaseColorY, this->coordBaseColorZ, this->coordBaseLength,
            this->coordBaseWidth);
    earthRotator->addChild(this->_drawables);

    osg::ref_ptr<osg::Geode> worldCoordSysGroup = new osg::Geode();
    drawCoordinateSystem(worldCoordSysGroup, this->coordBaseColorX,
            this->coordBaseColorY, this->coordBaseColorZ, this->coordBaseLength,
            this->coordBaseWidth);
    scene->addChild(worldCoordSysGroup);

    osg::ref_ptr<osg::Geode> eciGroup = new osg::Geode();
    drawCoordinateSystem(eciGroup, "fff0000f", "fff0000f", "fff0000f",
            100000000, this->coordBaseWidth);
    eciRotator->addChild(eciGroup);
    osg::Vec3d v;
    mapNode->getMap()->getSRS()->transformFromWorld(osg::Vec3d(1, 0, 0), v);
    //std::cout << "transformed vector: (" << v[0] << "," << v[1] << "," << v[2] << std::endl;
    eciRotator->setAttitude(osg::Quat(0, v));

    builtinOsgCanvas->setScene(scene);
#endif
}

OsgEarthScene* OsgEarthScene::getInstance() {
    if (instance == nullptr) {
        throw omnetpp::cRuntimeError("OsgRenderer::getInstance(): there is no "
                "OsgRenderer module in the network");
    }
    return instance;
}

void OsgEarthScene::refreshDisplay() const {

#ifdef WITH_OSG
    cJulian simTime = GlobalJulianDate::getInstance().currentSimTime();
    int year, month;
    double dayOfMonth;
    simTime.getComponent(&year, &month, &dayOfMonth);
    int days = (int) dayOfMonth;
    dayOfMonth -= (double) days;
#if OSGEARTH_VERSION_GREATER_OR_EQUAL(2, 6, 0)
    osgEarth::DateTime date(year, month, days, dayOfMonth * 24.0);
    skynode->setDateTime(date);
#else

  skynode->setDateTime(*year, *month, days, *dayOfMonth * 24.0);
#endif

    earthRotator->setAttitude(
            osg::Quat(inetu::rad(simTime.toGMST()).get(), osg::Vec3d(0, 0, 1)));
#endif
}

}  // namespace estnet
