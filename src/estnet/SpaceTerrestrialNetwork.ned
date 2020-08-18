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

package estnet;

import estnet.common.node.NodeRegistry;
import estnet.common.time.GlobalJulianDate;
import estnet.contactplan.ContactPlanVisualizer;
import estnet.contactplan.common.ContactPlanManager;
import estnet.node.errormodel.JammingStation;
import estnet.node.groundstation.GroundStation;
import estnet.node.groundstation.GroundLabel;
import estnet.node.satellite.Satellite;
import estnet.visualizer.OsgEarthScene;
import inet.environment.common.PhysicalEnvironment;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

//
// General network for space-terrestrial network simulations
//
network SpaceTerrestrialNetwork
{
    parameters:
        int numS; // number of satellites
        int numCg = default(0); // number of connected ground stations
        int numLabels = default(0); // number of ground labels
        int numJammer = default(0); // number of jamming stations 
        string mediumType;	// type of radio medium
        @display("bgb=650,500;bgg=100,1,grey95");

        @figure[title](type=label; pos=0,-1; anchor=sw; color=darkblue);

    submodules:
        physicalEnvironment: PhysicalEnvironment {
            @display("p=580, 425");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=580,200");
        }
        radioMedium: <mediumType> like IRadioMedium {
            @display("p=580,275");
        }
        osgEarthScene: OsgEarthScene {
            @display("is=vl;p=300,300");
        }
        nodeRegistry: NodeRegistry;
        globalJulianDate: GlobalJulianDate;
        contactPlanManager: ContactPlanManager {
            @display("p=300.384,114.912");
        }
        contactPlanVisualizer: ContactPlanVisualizer;
        g[numLabels]: GroundLabel {
            @display("p=100,100");
        }
        cg[numCg]: GroundStation {
            @display("p=100,200");
            nodeNo = numS + index + 1; // to keep nodeNo unique, we'll start counting after all satellites
        }
        sat[numS]: Satellite {
            @display("p=100,300");
            nodeNo = index + 1; // one-based counting to keep ION happy
        }
        jammer[numJammer]: JammingStation {
            @display("p=100,400");
        }
}
