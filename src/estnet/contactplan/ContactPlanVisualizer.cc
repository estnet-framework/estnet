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

#include "ContactPlanVisualizer.h"

#ifdef WITH_OSG
#include <osg/Depth>
#include <osg/LineWidth>
#include <osg/PolygonMode>
#include <osg/ValueObject>
#include <osgEarthUtil/LinearLineOfSight>
#include <osgUtil/UpdateVisitor>
#endif

#include <inet/common/geometry/object/LineSegment.h>
#include <inet/common/geometry/base/ShapeBase.h>
#include <inet/environment/common/PhysicalEnvironment.h>
#include <inet/common/geometry/shape/Sphere.h>

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/visualizer/OsgEarthScene.h"
#include "estnet/visualizer/OsgNode.h"
#include "estnet/visualizer/common/OsgUtils.h"
#include "estnet/common/StlUtils.h"


namespace estnet {

static ContactPlanVisualizer *instance;

Define_Module(ContactPlanVisualizer);

ContactPlanVisualizer::ContactPlanVisualizer() {
    if (instance != nullptr) {
        throw omnetpp::cRuntimeError(
                "There can be only one ContactPlanVisualizer instance in the network");
    }
    instance = this;
}

ContactPlanVisualizer::~ContactPlanVisualizer() {
#ifdef WITH_OSG
    this->_losNodes.clear();
#endif // WITH_OSG

    instance = nullptr;
}

ContactPlanVisualizer* ContactPlanVisualizer::getInstance() {
    if (instance == nullptr) {
        throw omnetpp::cRuntimeError(
                "ContactPlanVisualizer::getInstance(): there "
                        "is no ContactPlanVisualizer module in the "
                        "network");
    }
    return instance;
}

int ContactPlanVisualizer::numInitStages() const {
    return 3;
}

void ContactPlanVisualizer::initialize(int stage) {
#ifdef WITH_OSG
    if (stage == 0) {
        this->_cpManager = ContactPlanManager::getInstance();
        this->_satToSatColor = this->par("satToSatColor").stringValue();
        this->_satToSatColorOnPlan =
                this->par("satToSatColorOnPlan").stringValue();
        this->_satToSatWidth = this->par("satToSatWidth").doubleValue();
        this->_satToGroundColor = this->par("satToGroundColor").stringValue();
        this->_satToGroundColorOnPlan =
                this->par("satToGroundColorOnPlan").stringValue();
        this->_satToGroundWidth = this->par("satToGroundWidth").doubleValue();
        this->_show = this->par("show").boolValue();

    } else if (stage == 1) {
#ifdef WITH_OSG
        this->_scene = OsgEarthScene::getInstance()->getScene()->asGroup();
        this->_connections = new osg::Geode();
        this->_scene->addChild(this->_connections);
#endif // WITH_OSG
    } else if (stage == 2 && this->_show) {
        // note: satellites and ground stations must have been added by now
        NodeRegistry *nodeRegistry = NodeRegistry::getInstance();
        std::vector<Satellite*> satellites = nodeRegistry->getSatellites();
        std::vector<GroundStation*> groundStations =
                nodeRegistry->getGroundStations();
        auto mapNode = osgEarth::MapNode::findMapNode(this->_scene);

        for (const Satellite *satellite1 : satellites) {
            // create contact lines
            osg::Node *satellite1OsgNode =
                    dynamic_cast<OsgNode*>(satellite1->getSubmodule("osgNode"))->getLocatorNode();
            unsigned int satellite1NodeId =
                    satellite1->par("nodeNo").intValue();
            for (const GroundStation *groundStation : groundStations) {
                osg::Node *stationOsgNode =
                        dynamic_cast<OsgNode*>(groundStation->getSubmodule(
                                "osgNode"))->getLocatorNode();
                unsigned int stationNodeId =
                        groundStation->par("nodeNo").intValue();
                addLineOfSight(mapNode, satellite1OsgNode, stationOsgNode,
                        satellite1NodeId, stationNodeId, 0);
            }
            for (const Satellite *satellite2 : satellites) {
                osg::Node *satellite2OsgNode =
                        dynamic_cast<OsgNode*>(satellite2->getSubmodule(
                                "osgNode"))->getLocatorNode();
                unsigned int satellite2NodeId =
                        satellite2->par("nodeNo").intValue();
                addLineOfSight(mapNode, satellite1OsgNode, satellite2OsgNode,
                        satellite1NodeId, satellite2NodeId, 1);
            }
        }
    }
#endif // WITH_OSG
}

std::map<std::tuple<unsigned int, unsigned int>, contact_plan_entry_t*> ContactPlanVisualizer::getActiveContacts() const {
    std::vector<contact_plan_entry_t*> contacts =
            this->_cpManager->getActiveContacts();
    std::map<std::tuple<unsigned int, unsigned int>, contact_plan_entry_t*> contactMap;
    for (contact_plan_entry_t *contact : contacts) {
        contactMap.emplace(
                std::tuple<unsigned int, unsigned int>(contact->sourceNodeId,
                        contact->sinkNodeId), contact);
    }
    return contactMap;
}

#ifdef WITH_OSG
void ContactPlanVisualizer::addLineOfSight(osgEarth::MapNode *mapNode,
        osg::Node *a, osg::Node *b, unsigned int aNodeId, unsigned int bNodeId,
        int type) {
    auto los = new osgEarth::Util::LinearLineOfSightNode(mapNode);

    // not drawing the line of sight nodes' lines
    los->setGoodColor(osg::Vec4f(0, 0, 0, 0));
    los->setBadColor(osg::Vec4f(0, 0, 0, 0));

    auto stateSet = los->getOrCreateStateSet();
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    auto depth = new osg::Depth;
    depth->setWriteMask(false);
    stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

    los->setUserValue("type", type);
    los->setUserValue("aNodeId", aNodeId);
    los->setUserValue("bNodeId", bNodeId);

    los->setUpdateCallback(new osgEarth::Util::LineOfSightTether(a, b));
    los->setTerrainOnly(true); // so the dish model itself won't occlude

    this->_losNodes.push_back(los);
    this->_scene->addChild(los);
}
#endif // WITH_OSG

void ContactPlanVisualizer::refreshDisplay() const {
    if (!this->_show) {
        return;
    }
#ifdef WITH_OSG
    // los update callbacks are called during update traversal, so do it now
    osgUtil::UpdateVisitor updateVisitor;
    this->_scene->traverse(updateVisitor);

    // update line-of-sight lines (remove all, then add back current ones)
    this->_connections->removeDrawables(0,
            this->_connections->getNumDrawables());

    // grab active contacts from contact plan
    std::map<std::tuple<unsigned int, unsigned int>, contact_plan_entry_t*> activeContacts =
            this->getActiveContacts();

    int numSatToSat = 0;
    int numSatToGround = 0;
    int numSatToSatOnPlan = 0;
    int numSatToGroundOnPlan = 0;

    NodeRegistry *nodeRegistry = NodeRegistry::getInstance();


    for (auto losNode : this->_losNodes) {
        int type = -1;
        unsigned int aNodeId = 0, bNodeId = 0;
        losNode->getUserValue("type", type);
        losNode->getUserValue("aNodeId", aNodeId);
        losNode->getUserValue("bNodeId", bNodeId);



        if (losNode->getHasLOS()) {
            auto start = losNode->getStartWorld();
            auto end = losNode->getEndWorld();

            // doing additional check for Earths intersection
            // as with large distances, there might be a double overflow
            inet::physicalenvironment::PhysicalEnvironment *pEnv =
                    omnetpp::check_and_cast<
                            inet::physicalenvironment::PhysicalEnvironment*>(
                            this->getSystemModule()->getSubmodule(
                                    "physicalEnvironment"));
            const inet::ShapeBase *shape = pEnv->getObject(0)->getShape();
            auto earth = const_cast<inet::Sphere*>(omnetpp::check_and_cast<
                    const inet::Sphere*>(shape));

            auto p1 = nodeRegistry->getNode(aNodeId)->getMobility()->getCurrentPosition();
            auto p2 = nodeRegistry->getNode(bNodeId)->getMobility()->getCurrentPosition();
            // the 10e6 is a scaling factor to prevent the overflow
            inet::LineSegment line = inet::LineSegment(
                    p1 / 1000000, p2 / 1000000);
            inet::Coord c1, c2, c3, c4;
            earth->setRadius(earth->getRadius() / 1000000);
            bool recheck = !earth->computeIntersection(line, c1, c2, c3, c4);
            earth->setRadius(earth->getRadius() * 1000000);


            bool isOnContactPlan = contains(activeContacts,
                    std::tuple<unsigned int, unsigned int>(aNodeId, bNodeId));

            osgEarth::Color lineColor;
            double lineWidth = 0;
            bool draw = true;
            if (type == 0 && !isOnContactPlan) {
                draw = !this->_satToGroundColor.empty();
                lineColor = osgEarth::Color(this->_satToGroundColor);
                lineWidth = this->_satToGroundWidth;
                numSatToGround++;
            } else if (type == 0 && isOnContactPlan) {
                draw = !this->_satToGroundColorOnPlan.empty();
                lineColor = osgEarth::Color(this->_satToGroundColorOnPlan);
                lineWidth = this->_satToGroundWidth;
                numSatToGround++;
                numSatToGroundOnPlan++;
            } else if (type == 1 && !isOnContactPlan) {
                draw = !this->_satToSatColor.empty();
                lineColor = osgEarth::Color(this->_satToSatColor);
                lineWidth = this->_satToSatWidth;
                numSatToSat++;
            } else if (type == 1 && isOnContactPlan) {
                draw = !this->_satToSatColorOnPlan.empty();
                lineColor = osgEarth::Color(this->_satToSatColorOnPlan);
                lineWidth = this->_satToSatWidth;
                numSatToSat++;
                numSatToSatOnPlan++;
            }
            if (draw && recheck) {
                this->_connections->addDrawable(
                        createLineBetweenPoints(start, end, lineWidth,
                                lineColor));
            }
        }
    }

    // EV << "Active connections: " << numSatToSat << " (" << numSatToSatOnPlan <<
    // " on plan) sat-to-sat and " << numSatToGround << " (" <<
    // numSatToGroundOnPlan << " on plan) sat-to-ground \n";

#endif // WITH_OSG

}

}  // namespace estnet
