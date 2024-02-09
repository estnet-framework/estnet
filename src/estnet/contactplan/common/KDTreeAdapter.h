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

#ifndef UTILS_KDTREEADAPTER_H_
#define UTILS_KDTREEADAPTER_H_

#include <math.h>
#include <map>
#include <algorithm>

#include <inet/common/INETMath.h>
#include <inet/physicallayer/contract/packetlevel/IRadio.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskScalarReceiver.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskScalarTransmitter.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskRadio.h>
#include <inet/physicallayer/common/packetlevel/RadioMedium.h>
#include <inet/common/geometry/common/Coord.h>

#include "nanoflann.hpp"
#include "estnet/node/base/NodeBase.h"
#include "estnet/common/node/NodeRegistry.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/satellite/propagator/position/PositionPropagatorKepler.h"

using namespace std;
using namespace inet;
using namespace physicallayer;
using namespace units::constants;
using namespace nanoflann;

namespace estnet {

typedef std::vector<const IRadio*> radioList;

// struct representing node in kd-tree
template<typename T, class nodeType>
struct node {
    //storing coordinates of current position
    T x, y, z;
    unsigned int nodeId;
    nodeType radio;
};

//struct needed by nanoflann kd-tree as datastruct
template<typename T, class nodeType>
struct nodePoints {
    std::map<unsigned int, node<T, nodeType>> nodes = std::map<unsigned int,
            node<T, nodeType>>();

    inline T kdtree_get_pt(const size_t idx, int dim) const {
        unsigned int id = static_cast<unsigned int>(idx);
        node<T, nodeType> newNode = nodes.at(id);
        if (dim == 0)
            return newNode.x;
        else if (dim == 1)
            return newNode.y;
        else
            return newNode.z;
    }

    inline size_t kdtree_get_point_count() const {
        return nodes.size();
    }

    template<class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
        return false;
    }
};

/**
 * class for saving radios and nodes in a k-d-Tree
 * using efficient radius search for contact planning
 * based on nanoflann
 */
class ESTNET_API KDTreeAdapter {
    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<long long, nodePoints<long long, const IRadio*> >,
            nodePoints<long long, const IRadio*>, 3 /* dim */, long long> radio_kd_tree_t; ///< KDTreeAdaptor definition for radios
public:
    ~KDTreeAdapter() {
        delete currentTree;
        delete radioNodePtsPtr;
    }
    /** @brief generate a node storing a radio with current position and node-ID*/
    template<typename T>
    node<T, const IRadio*> generateNode(const IRadio *iRadio) {
        const Radio *radio = check_and_cast<const Radio*>(iRadio);
        auto nodeB = check_and_cast<const NodeBase*>(
                radio->getParentModule()->getParentModule()->getParentModule());
        Coord coordinates = nodeB->getMobility()->getCurrentPosition();
        // the node is fed with the radio's position and node number of the belonging satellite
        node<T, const IRadio*> newNode;
        newNode.x = coordinates.x / 1000;
        newNode.y = coordinates.y / 1000;
        newNode.z = coordinates.z / 1000;
        newNode.radio = radio;
        newNode.nodeId = nodeB->getNodeNo();
        return newNode;
    }

    /** @brief feeds the DatasetAdapter with all radios in nodes*/
    template<typename T>
    void generateCoordinates(nodePoints<T, const IRadio*> *&radioNodePts,
            const radioList radios) {
        for (auto iRadio : radios) {
            //create node with radios position
            node<T, const IRadio*> newNode = generateNode<T>(iRadio);
            radioNodePts->nodes.insert(
                    std::pair<T, node<T, const IRadio*>>(newNode.nodeId - 1,
                            newNode));
        }
    }

    /** @brief generate the kd-Tree with current data of the radios*/
    template<typename num_t>
    void createKDRadioTree(const radioList radios) {
        //free memory and create new empty list
        if (currentTree != nullptr) {
            delete currentTree;
            delete radioNodePtsPtr;
        }
        radioNodePtsPtr = new nodePoints<num_t, const IRadio*>();
        generateCoordinates<num_t>(radioNodePtsPtr, radios);
        currentTree = new radio_kd_tree_t(3 /*dim*/, *radioNodePtsPtr,
                KDTreeSingleIndexAdaptorParams(30 /* max leaf */));
        currentTree->buildIndex();
    }

    /** @brief compute the maximum communication range based on the APSK model or the geometric constraints */
    unsigned int computeCommRadius(const IRadio *iRadio,
            const IRadio *iRadioPartner, RadioMedium* radioMedium);

    /** @brief compute the maximum communication range im kilometer based on the geometric constraints */
    unsigned int calcGeomConstrRadius(const Radio *radio,
            const Radio *radioPartner);

    /** @brief does an unsorted radius search starting at the given radio and puts result in ret_matches*/
    template<typename num_t>
    void unsortedRadiusSearch(int transmitterId,
            std::vector<std::pair<long long, long long> > &ret_matches,
            long long search_radius) {
        nanoflann::SearchParams params;
        params.sorted = false; //sorted is not required
        int id = transmitterId - 1;
        search_radius += 1 + search_radius * 0.01; //add 1 % on radius for safety (casts from double to int and calculation inaccuracies)
        long long x = currentTree->dataset.kdtree_get_pt(id, 0);
        long long y = currentTree->dataset.kdtree_get_pt(id, 1);
        long long z = currentTree->dataset.kdtree_get_pt(id, 2);
        const long long query_pt[3] = { x, y, z };
        currentTree->radiusSearch(&query_pt[0], search_radius * search_radius,
                ret_matches, params);

    }

    
private:
    radio_kd_tree_t *currentTree = nullptr; ///< the radio kdtree is stored in this variable
    nodePoints<long long, const IRadio*> *radioNodePtsPtr = nullptr; ///< list containing all radios with position as node

};

}  // namespace estnet

#endif /* UTILS_KDTREEADAPTER_H_ */
