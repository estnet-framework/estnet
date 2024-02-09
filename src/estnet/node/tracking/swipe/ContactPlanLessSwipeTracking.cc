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

#include "ContactPlanLessSwipeTracking.h"

#include <iterator>

#include "estnet/common/ModuleAccess.h"
#include "estnet/common/node/NodeRegistry.h"
#include "estnet/mobility/satellite/SatMobility.h"


namespace estnet {

Define_Module(ContactPlanLessSwipeTracking);

inet::Quaternion ContactPlanLessSwipeTracking::getNewOrientation() {
  auto currentSimTime = omnetpp::simTime().dbl();

  auto start = this->contactPeriods[currentContact];
  auto end = this->contactPeriods[currentContact + 1];

  double interp = (currentSimTime - start) / (end - start);

  interp += std::sin(interp * 2.0 * M_PI) * this->trackSinAmount;
  interp = std::min(std::max(0.0, interp), 1.0);

  auto nodeRegistry = estnet::NodeRegistry::getInstance();
  auto satellites = nodeRegistry->getSatellites();
  auto satCount = nodeRegistry->getNumSatellites();
  double satIndex = (satCount - 1) * interp;

  auto groundstation =
      check_and_cast<estnet::GroundStation *>(this->getParentModule());
  auto mobility = groundstation->getMobility();
  auto pos = mobility->getCurrentPosition();

  auto sat1 = satellites[(int)satIndex];
  auto sat1Mobility = dynamic_cast<estnet::SatMobility *>(sat1->getMobility());
  auto sat1Pos = sat1Mobility->getCurrentPositionWithoutSignal();
  auto sat1Direction =
      inet::Quaternion::rotationFromTo(inet::Coord::X_AXIS, sat1Pos - pos);

  auto sat2 = satellites[std::min((int)satIndex + 1, satCount - 1)];
  auto sat2Mobility = dynamic_cast<estnet::SatMobility *>(sat2->getMobility());
  auto sat2Pos = sat2Mobility->getCurrentPositionWithoutSignal();
  auto sat2Direction =
      inet::Quaternion::rotationFromTo(inet::Coord::X_AXIS, sat2Pos - pos);

  return inet::Quaternion::slerp(sat1Direction, sat2Direction,
                                 fmod(satIndex, 1));
}

void ContactPlanLessSwipeTracking::initialize() {
  estnet::INodeTracking::initialize();

  this->_swipeStart = new omnetpp::cMessage("startSwipeTimer");
  this->_swipeStop = new omnetpp::cMessage("stopSwipeTimer");

  std::istringstream iss(this->par("gsContactPeriods").stringValue());
  std::transform(std::istream_iterator<std::string>(iss),
                 std::istream_iterator<std::string>(),
                 std::back_inserter(this->contactPeriods),
                 [](const std::string &s1) -> double {
                   double x;
                   std::stringstream ss(s1);
                   ss >> x;
                   return x;
                 });
  
  this->trackSinAmount = this->par("trackSinAmount");

  this->scheduleNextSwipe();
}

void ContactPlanLessSwipeTracking::handleMessage(cMessage *message) {
  if (message == this->_swipeStart) {
    this->start();
  } else if (message == this->_swipeStop) {
    this->stop();
    currentContact++;
    this->scheduleNextSwipe();
  }
}

void ContactPlanLessSwipeTracking::finish() {
  cancelAndDelete(this->_swipeStart);
  cancelAndDelete(this->_swipeStop);
}

void ContactPlanLessSwipeTracking::scheduleNextSwipe() {
  if (this->contactPeriods.size() >= currentContact * 2 + 2) {
    this->scheduleAt(this->contactPeriods[currentContact * 2 + 0],
                     this->_swipeStart);
    this->scheduleAt(this->contactPeriods[currentContact * 2 + 1],
                     this->_swipeStop);
  }
}

} // namespace estnet
