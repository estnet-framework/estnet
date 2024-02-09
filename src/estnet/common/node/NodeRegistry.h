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

#ifndef __CONTACTPLANS_NODE_REGISTRY_H__
#define __CONTACTPLANS_NODE_REGISTRY_H__

#include <omnetpp.h>

#include "../../node/groundstation/GroundStation.h"
#include "estnet/common/ESTNETDefs.h"
#include "estnet/node/satellite/Satellite.h"

namespace estnet {

/**
 * Registry of all comunicating
 * nodes in the simulation.
 * Keeps track of all satellites
 * and ground stations in the simulation.
 */
class ESTNET_API NodeRegistry : public omnetpp::cSimpleModule {
private:
  std::vector<Satellite *> _satellites;
  std::vector<GroundStation *> _connectedGroundStations;

public:
  /** @brief should only be invoked by the simulation, will throw exception afterwards */
  NodeRegistry();
  /** @brief cleanup */
  virtual ~NodeRegistry();
  /** @brief returns the singleton instance of the NodeRegistry */
  static NodeRegistry *getInstance();

  /** @brief adds a satellite to the registry.
   *  needs to be called exactly in initialize stage 1 */
  void addSatellite(Satellite *p);
   /** @brief adds a ground station to the registry.
   *  needs to be called exactly in initialize stage 1 */
  void addGroundStation(
      GroundStation *p);

  /** @brief returns a list of all satellites in the simulation */
  std::vector<Satellite *> getSatellites() const;
  /** @brief returns the number of satellites in the simulation */
  int getNumSatellites() const;
  /** @brief returns the satellite with the given node no or null */
  Satellite *getSatellite(unsigned int nodeId) const;
  /** @brief checks if the given node no is a satellite */
  bool isSatellite(unsigned int nodeId) const;

  /** @brief returns a list of all ground stations in the simulation */
  std::vector<GroundStation *> getGroundStations() const;
  /** @brief returns the number of ground stations in the simulation */
  int getNumGroundStations() const;
  /** @brief returns the ground station with the given node no or null */
  GroundStation *getGroundStation(unsigned int nodeId) const;
  /** @brief checks if the given node no is a ground station */
  bool isGroundStation(unsigned int nodeId) const;

  /** @brief returns a list of all nodes in the simulation */
  std::vector<NodeBase *> getNodes() const;
  /** @brief returns the number of nodes in the simulation */
  int getNumNodes() const;
  /** @brief returns the node with the given node no or null */
  NodeBase *getNode(unsigned int nodeId) const;
  /** @brief checks if the given node no is a node */
  bool isNode(unsigned int nodeId) const;
};

}  // namespace estnet

#endif
