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
#include <omnetpp/osgutil.h>

#include <osg/Image>
#include <osg/LineWidth>
#include <osg/Node>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/Texture>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/Geometry>
#include <osgEarth/Version>

#endif // WITH_OSG

#include "GroundLabel.h"

namespace estnet {

Define_Module(GroundLabel)

void GroundLabel::initialize(int stage) {
    switch (stage) {
    case 0:
        timeStep = par("timeStep");
        modelURL = par("modelURL").stringValue();
        modelScale = par("modelScale");
        labelColor = par("labelColor").stringValue();
        longitude = par("longitude");
        latitude = par("latitude");
        break;
#ifdef WITH_OSG

    case 1:

        // scene is initialized in stage 0 so we have to do our init in stage 1
        auto scene = OsgEarthScene::getInstance()->getScene();
        mapNode = osgEarth::MapNode::findMapNode(scene);

        // build up the node representing this module
        // an ObjectLocatorNode allows positioning a model using world coordinates
        geoTransform = new osgEarth::GeoTransform();
        locatorNode = new osg::PositionAttitudeTransform();
        auto modelNode = osgDB::readNodeFile(modelURL);

        auto objectNode = new omnetpp::cObjectOsgNode(this);
#if OSGEARTH_VERSION_GREATER_OR_EQUAL(2, 10, 0)
    mapNode->addChild(geoTransform);
#else
        mapNode->getModelLayerGroup()->addChild(geoTransform);
#endif
        geoTransform->addChild(locatorNode);
        locatorNode->addChild(objectNode);
        objectNode->addChild(modelNode);

        geoTransform->setPosition(
                osgEarth::GeoPoint(mapNode->getMapSRS(), longitude, latitude,
                        altitude));
        locatorNode->setAttitude(osg::Quat(0, osg::Vec3d(0, 0, 1)));

        locatorNode->getOrCreateStateSet()->setAttributeAndModes(
                new osg::Program(),
                osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
        locatorNode->getOrCreateStateSet()->setMode(
        GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

        // scale and rotate the model if necessary
        locatorNode->setScale(osg::Vec3d(modelScale, modelScale, modelScale));

        // set the name label if the color is specified
        if (!labelColor.empty()) {
            osgEarth::Symbology::Style labelStyle;
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->alignment() =
                    osgEarth::Symbology::TextSymbol::ALIGN_CENTER_TOP;
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->declutter() =
                    true;
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->pixelOffset() =
                    osg::Vec2s(0, 40);
            labelStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->fill()->color() =
                    osgEarth::Color(labelColor);
            locatorNode->addChild(
                    new osgEarth::Annotation::LabelNode(par("label"),
                            labelStyle));
        }

        // add the locator node to the scene
        mapNode->addChild(locatorNode);

        // position the nodes, so we will see them at correct position right after
        // initialization
        refreshVisuals();

        break;
#endif // WITH_OSG
    }
}

void GroundLabel::refreshVisuals() {
#ifdef WITH_OSG
    locatorNode->setPosition(osg::Vec3d(longitude, latitude, altitude));
#endif // WITH_OSG
}

void GroundLabel::handleMessage(omnetpp::cMessage *msg) {
    // nothing to do here
}

}  // namespace estnet
