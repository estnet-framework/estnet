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

package estnet.power;

import estnet.power.contract.IConsumerModule;
import estnet.power.contract.ISolarPanel;
import inet.power.contract.IEpEnergyStorage;
import inet.power.management.SimpleEpEnergyManagement;

//
// Module that contains all energy producer and consumer that are not simulated anywhere else
//
module EnergyModule extends SimpleEpEnergyManagement
{
    parameters:

        int numSolarPanel = default(0);  			       // Amount of SolarPanels
        int numConsumerModules = default(1);  		       // Amount of Consumer
        string solarPanelType = default("SolarPanelBase"); // Charging Device Type
        string batteryType = default("SimpleEpBattery");   // Battery Type
        string consumerModuleType = default("CubeSatConsumer");   // consumer Type
        bool supplyConsumerDirectly = default(false);      // decides whether power is directly delivered to consumers or always to battery
        double sunAngle @unit(deg) = default(0deg); // describes in whicht time of the year the simulation takes place

        @class(EnergyModule);
    submodules:
        //Energy creators
        solarpanel[numSolarPanel]: <solarPanelType> like ISolarPanel {
            @display("p=243,52");
        }
        //Energy storage
        battery: <batteryType> like IEpEnergyStorage {
            @display("p=243,246");
        }
        //Energy consumers
        consumer[numConsumerModules]: <consumerModuleType> like IConsumerModule {
            @display("p=243,143");
        }
}
