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

#include "NodeRegistry.h"
#include "estnet/common/StlUtils.h"

namespace estnet {

static NodeRegistry *instance;

Define_Module(NodeRegistry);

NodeRegistry::NodeRegistry() {
  if (instance != nullptr) {
    throw omnetpp::cRuntimeError(
        "There can be only one NodeRegistry instance in the network");
  }
  instance = this;
}

NodeRegistry::~NodeRegistry() { instance = nullptr; }

NodeRegistry *NodeRegistry::getInstance() {
  if (instance == nullptr) {
    throw omnetpp::cRuntimeError("NodeRegistry::getInstance(): there is no NodeRegistry "
                        "module in the network");
  }
  return instance;
}

void NodeRegistry::addSatellite(Satellite *p) {
  this->_satellites.push_back(p);
}

void NodeRegistry::addGroundStation(GroundStation *p) {
  this->_connectedGroundStations.push_back(p);
}

std::vector<Satellite *> NodeRegistry::getSatellites() const {
  return this->_satellites;
}

int NodeRegistry::getNumSatellites() const { return this->_satellites.size(); }

Satellite *NodeRegistry::getSatellite(unsigned int nodeId) const {
  for (Satellite *s : this->_satellites) {
    if (s->getNodeNo() == nodeId) {
      return s;
    }
  }
  return nullptr;
}

bool NodeRegistry::isSatellite(unsigned int nodeId) const {
  return this->getSatellite(nodeId) != nullptr;
}

std::vector<GroundStation *>
NodeRegistry::getGroundStations() const {
  return this->_connectedGroundStations;
}

int NodeRegistry::getNumGroundStations() const {
  return this->_connectedGroundStations.size();
}

GroundStation *
NodeRegistry::getGroundStation(unsigned int nodeId) const {
  for (GroundStation *s : this->_connectedGroundStations) {
    if (s->getNodeNo() == nodeId) {
      return s;
    }
  }
  return nullptr;
}

bool NodeRegistry::isGroundStation(unsigned int nodeId) const {
  return this->getGroundStation(nodeId) != nullptr;
}

std::vector<NodeBase *> NodeRegistry::getNodes() const {
  std::vector<NodeBase *> rtn;
  std::vector<Satellite *> sats = this->getSatellites();
  std::vector<GroundStation *> gss =
      this->getGroundStations();
  rtn.insert(rtn.end(), sats.begin(), sats.end());
  rtn.insert(rtn.end(), gss.begin(), gss.end());
  return rtn;
}

int NodeRegistry::getNumNodes() const {
  return this->_satellites.size() + this->_connectedGroundStations.size();
}

NodeBase *NodeRegistry::getNode(unsigned int nodeId) const {
  NodeBase *rtn = this->getSatellite(nodeId);
  if (rtn == nullptr) {
    rtn = this->getGroundStation(nodeId);
  }
  return rtn;
}

bool NodeRegistry::isNode(unsigned int nodeId) const {
  return this->getNode(nodeId) != nullptr;
}

}  // namespace estnet
