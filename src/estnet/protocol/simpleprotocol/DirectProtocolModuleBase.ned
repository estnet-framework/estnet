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

package estnet.protocol.simpleprotocol;

import estnet.protocol.base.ProtocolModuleBase;

// Implementation ~ProtocolModuleBase for the simple protocol.
// Just converts received app packets to radio frames and
// received radio frames to app packets.
//
module DirectProtocolModuleBase extends ProtocolModuleBase
{
    parameters:
        bool useBroadcasts; // whether to use broadcasts
        @class(DirectProtocolModuleBase);
}
