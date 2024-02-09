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

#ifndef __OSGRENDERER_H__
#define __OSGRENDERER_H__

#ifdef WITH_OSG
#include <osg/Node>
#include <osgEarth/MapNode>
#include <osgEarth/Version>

#if OSGEARTH_VERSION_GREATER_OR_EQUAL(2, 6, 0)
#include <osgEarthUtil/Sky>
#else
#include <osgEarthUtil/SkyNode>
#endif

#endif

#include "estnet/common/ESTNETDefs.h"
#include "estnet/common/time/GlobalJulianDate.h"

namespace estnet {

/**
 * Initialize global 3d canvas and load the configured earth model file.
 */
class ESTNET_API OsgEarthScene: public omnetpp::cSimpleModule {
protected:
    static OsgEarthScene *instance;
#ifdef WITH_OSG
    osg::ref_ptr<osg::Group> scene;
    osgEarth::Util::SkyNode *skynode;
    osg::PositionAttitudeTransform *earthRotator = nullptr;
    osg::ref_ptr<osg::Geode> _drawables;
    std::string coordBaseColorX;
    std::string coordBaseColorY;
    std::string coordBaseColorZ;
    double coordBaseLength;
    double coordBaseWidth;
#endif

public:
    OsgEarthScene();
    virtual ~OsgEarthScene();

    /**
     * Accessor to Singleton instance
     * @return OsgEarthScene: singleton instance of OsgEarthScene
     */
    static OsgEarthScene* getInstance();
#ifdef WITH_OSG
    /**
     * Access to he scene, which is the group holding the osg stuff
     * @return Group: group that holds the osg scene
     */
    virtual osg::Group* getScene() {
        return scene;
    }
#endif

protected:
    /**
     * Initialization of the OSG visualization
     */
    virtual void initialize() override;

    /**
     * Updates all visualized components in the OSG scene,
     * i.e the earth's rotation and sunlight
     *
     */
    virtual void refreshDisplay() const override;
};

}  // namespace estnet

#endif
