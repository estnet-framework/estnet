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

#ifndef __CONTACTPLANS_VISUALIZER_H__
#define __CONTACTPLANS_VISUALIZER_H__

#include <omnetpp.h>

#ifdef WITH_OSG
#include <osg/Node>
#include <osgEarth/MapNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/Style>
#include <osgEarthUtil/LineOfSight>
#include <osgEarthUtil/LinearLineOfSight>
#endif

#include "common/ContactPlanManager.h"
#include "common/Contacts.h"

namespace estnet {

/**
 * Visualizes lines of sight between nodes
 * in the simulation.
 */
class ESTNET_API ContactPlanVisualizer: public omnetpp::cSimpleModule {
private:
    ContactPlanManager *_cpManager;
    std::string _satToSatColor;
    std::string _satToSatColorOnPlan;
    std::string _satToGroundColor;
    std::string _satToGroundColorOnPlan;
    bool _show;
#ifdef WITH_OSG
    double _satToSatWidth;
    double _satToGroundWidth;
    std::vector<osgEarth::Util::LinearLineOfSightNode*> _losNodes;
    osg::Group *_scene;
    osg::ref_ptr<osg::Geode> _connections;

    /** @brief adds a line-of-sight node to the OSG scene */
    void addLineOfSight(osgEarth::MapNode *mapNode, osg::Node *a, osg::Node *b,
            unsigned int aNodeId, unsigned int bNodeId, int type);
#endif
    /** @brief returns all currently active contacts from the active contact plan */
    std::map<std::tuple<unsigned int, unsigned int>, contact_plan_entry_t*>
    getActiveContacts() const;

protected:
    /** @brief returns number of initalization stages */
    virtual int numInitStages() const override;
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /**
     * This method is invoked by runtime GUIs on every display refresh,
     * and can be overridden if the figure needs to be able to update
     * its contents dynamically (self-refreshing figures.) The changes
     * done inside this method should be restricted to this figure
     * and its subfigure tree.
     */
    virtual void refreshDisplay() const override;

public:
    /** @brief should only be invoked by the simulation, will throw exception afterwards */
    ContactPlanVisualizer();
    /** @brief cleanup */
    virtual ~ContactPlanVisualizer();
    /** @brief returns the single instance of the visualizer */
    static ContactPlanVisualizer* getInstance();
};

}  // namespace estnet

#endif
