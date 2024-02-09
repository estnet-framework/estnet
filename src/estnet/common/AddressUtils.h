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

#ifndef __UTILS_ADDRESS_UTILS_H__
#define __UTILS_ADDRESS_UTILS_H__

#include <inet/linklayer/common/MacAddress.h>
#include <inet/networklayer/contract/ipv4/Ipv4Address.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/** @brief Returns the MAC address of a given node and radio no */
inet::MacAddress getMacAddressOfNode(unsigned int nodeNo, unsigned int radioNo);
/** @brief Returns the IP address of a given node and radio no */
inet::Ipv4Address getIpAddressOfNode(unsigned int nodeNo, unsigned int radioNo);

}  // namespace estnet

#endif
