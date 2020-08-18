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

#ifndef __estnet_OSGNODE_H_
#define __estnet_OSGNODE_H_

#include <omnetpp.h>
#ifdef WITH_OSG
#include <osgEarth/MapNode>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/GeoTransform>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/FeatureNode>
#endif

#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/terrestrial/StaticTerrestrialMobility.h"

namespace estnet {

/**
 * Implements OsgNode.ned object
 */
class ESTNET_API OsgNode: public omnetpp::cSimpleModule {
protected:
    double timeStep;        ///< temporal simulation resolution
    std::string labelColor; ///< color of the model label
    std::string modelURL;   ///< 3D satellite icon path
    double modelScale;
    std::string orbitColor;
    double orbitResolution;
    double orbitUpdateFreq;
    double orbitNextUpdate;
    std::string satConeColor;
    double satConeScaling;
    std::string coordBaseColorX;
    std::string coordBaseColorY;
    std::string coordBaseColorZ;
    double coordBaseLength;
    double coordBaseWidth;
    SatMobility *satMobility = nullptr; ///< SatMobility module as position source
    StaticTerrestrialMobility *staticTerrestrialMobility = nullptr; ///< SatMobility module as position source

#ifdef WITH_OSG
    osg::observer_ptr<osgEarth::MapNode> mapNode = nullptr; // the node containing the osgEarth data
    osgEarth::GeoTransform *geoTransform = nullptr; // osgEarth node for 3D visualization
    osg::ref_ptr<osg::Geode> _drawables;
    osg::MatrixTransform *_modelMatrixTransform;
    osg::ref_ptr<osg::Geode> _orbitGeode;
#endif

    /**
     * Initializes all the orbit-parameters and the 3D anmimation.
     * Schedules the first initial movement
     * @param stage of the initialization (2 steps needed)
     */
    virtual void initialize(int stage) override;

    /**
     * @return needed initialization steps
     */
    virtual int numInitStages() const override
    {
        return 2;
    }

    /**
     * from cModule
     * handler for incomming messages. in this case as timer for the next position
     * update
     * @param msg incomming message
     */
    virtual void handleMessage(omnetpp::cMessage *msg) override;

    /**
     * from cModule
     * to refresh all visualizations
     */
    virtual void refreshDisplay() const override;

    /**
     * draw the orbit shape
     */
    void drawOrbit(double time) const;

    /**
     * draw satellite commuication cone
     */
    void drawSatelliteCone() const;

public:
#ifdef WITH_OSG
    osgEarth::GeoTransform* getLocatorNode() const {
        return geoTransform;
    }
    ;
#endif
};

}  // namespace estnet

#endif
