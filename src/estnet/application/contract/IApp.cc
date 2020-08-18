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

#include "IApp.h"

unsigned int estnet::IApp::getNodeId() const {
    return this->_nodeId;
}

unsigned int estnet::IApp::getId() const {
    return this->_id;
}

unsigned int estnet::IApp::getPktSent() const {
    return this->_numSent;
}

unsigned int estnet::IApp::getPktRcvd() const {
    return this->_numReceived;
}
