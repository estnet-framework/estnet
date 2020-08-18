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

package estnet.power.consumer;

import estnet.power.consumer.modules.ADS;
import estnet.power.consumer.modules.HPCU;
import estnet.power.consumer.modules.IMA;
import estnet.power.consumer.modules.NAV;
import estnet.power.consumer.modules.OCS;
import estnet.power.consumer.type.ConstantConsumer;
import estnet.power.contract.IConsumerModule;


//
// Module which holds all the different consumer modules
// These are typical cubesat consumer
// Acts like as default consumer configuration
//
module CubeSatConsumer like IConsumerModule
{
    parameters:
        int numConstant = default(0); // number of constant consumers
        int numOcs = default(0);	// number of orbit control system consumers
        int numNav = default(0);	// number of navigation device consumers
        int numHpcu = default(0);	// number of computing device consumers
        int numIma = default(0);	// number of imaging devices/sensors
        int numAds = default(0);	// number of attidute determination devices

    submodules:
        // On-Board Computer (OBC), Electronic Power System(EPS),..
        constantConsumers[numConstant]: ConstantConsumer;
        // Orbit Control System
        OCS[numOcs]: OCS;
        // Navigation System
        NAV[numNav]: NAV;
        // Image sensors
        IMA[numIma]: IMA;
        // High Performance Computing Unit
        HPCU[numHpcu]: HPCU;
        // Attitude Determination System
        ADS[numAds]: ADS;
}
