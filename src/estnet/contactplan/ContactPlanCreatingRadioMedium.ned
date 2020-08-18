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

package estnet.contactplan;

import inet.physicallayer.common.packetlevel.RadioMedium;


//
// Wrapper around a real radio medium implementation
// which periodically checks for contacts and interferences.
// At the end of the simulation the resulting contact &
// intereference plan is written to files.
//
module ContactPlanCreatingRadioMedium extends RadioMedium
{
    parameters:
        int checkInterval @unit(s) = default(10s);            // interval at which contacts will be checked
        // satellite contacts are only checked in one directions
        // and automatically assumed to be bidirectional
        bool assumeBidirectionalSatContacts = default(false);  // whether satellite contact are considered bidirectional
        bool considerAddedInterferences = default(true);	// whetheradded interference should be considered
        bool buildInterferencePlan = default(true);	// whether an interference plan should be created
        // not used, realRadioMedium decides what to use, but needed to be created properly
        propagation.typename = default("ConstantSpeedPropagation");
        pathLoss.typename = default("FreeSpacePathLoss");
        analogModel.typename = default("ScalarAnalogModel");
        backgroundNoise.typename = default("IsotropicScalarBackgroundNoise");
        backgroundNoise.power = default(-194dBm);

        @class(ContactPlanCreatingRadioMedium);
}
