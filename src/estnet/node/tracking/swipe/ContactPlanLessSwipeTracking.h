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

#ifndef __ESTNET_ICONTACTPLANLESSSWIPETRACKING_H_
#define __ESTNET_ICONTACTPLANLESSSWIPETRACKING_H_

#include "estnet/node/tracking/contract/INodeTracking.h"

namespace estnet {

/**
 * Swipe algorithm which doesn't use contactplans, but gsContactPeriods instead.
 */
class ESTNET_API ContactPlanLessSwipeTracking : public estnet::INodeTracking {
public:
  virtual inet::Quaternion getNewOrientation() override;

protected:
  /** @brief initialization */
  virtual void initialize() override;
  /** @brief scheduled self-messages receiver function */
  virtual void handleMessage(omnetpp::cMessage *message) override;
  /** @brief cleanup messages */
  virtual void finish() override;

private:
  void scheduleNextSwipe();

  omnetpp::cMessage *_swipeStart;
  omnetpp::cMessage *_swipeStop;
  size_t currentContact = 0;
  std::vector<double> contactPeriods;
  double trackSinAmount;
};

} // namespace estnet

#endif
