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

#ifndef GROUNDSTATION_H_
#define GROUNDSTATION_H_

#ifdef WITH_OSG
#include <osgEarth/MapNode>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/GeoTransform>
#endif // WITH_OSG

#include "estnet/visualizer/OsgEarthScene.h"
#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/***
 * represents points on the earth surface
 * just taking coordinates (lat, lon, alt)
 * visualizing in 3D model
 */
class ESTNET_API GroundLabel: public omnetpp::cSimpleModule {
protected:
    // configuration
    double timeStep; //< Time Steps between Position Update
    unsigned int trailLength;
    std::string labelColor;
    std::string rangeColor;
    std::string trailColor;
    std::string modelURL; // 3D model
    double modelScale;    // 3D model scaling factor

#ifdef WITH_OSG
    osg::observer_ptr<osgEarth::MapNode> mapNode = nullptr; //< the node containing the osgEarth data

    osgEarth::GeoTransform *geoTransform = nullptr;
    osg::PositionAttitudeTransform *locatorNode = nullptr; // osgEarth node for 3D visualization
#endif // WITH_OSG

    double longitude = -70, latitude = 40, altitude = 400; // default coordinates

public:
#ifdef WITH_OSG
    osg::Vec3d getPosition() {
        return osg::Vec3d(longitude, latitude, altitude);
    } //< returns current position
    osg::Node* getLocatorNode() {
        return locatorNode;
    }
    ; // 3D visualisation node
#endif // WITH_OSG

protected:
    /***
     * two-step initialization
     * @param stage initialization stage
     */
    virtual void initialize(int stage) override;
    /***
     * method is mandatory for multi-stage initialization
     * @return two steps for the ground station
     */
    virtual int numInitStages() const override {
        return 2;
    }
    /***
     * mandatory in ometpp but not used yet (no message handling)
     */
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    /***
     * refreshes the 3D positioning
     */
    virtual void refreshVisuals();
};

}  // namespace estnet

#endif /* GROUNDSTATION_H_ */
