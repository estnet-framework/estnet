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

package estnet.mobility.satellite;

import inet.mobility.contract.IMobility;
import estnet.mobility.satellite.contract.IAttitudePropagator;
import estnet.mobility.satellite.contract.IPositionPropagator;

//
// The SatMobility module handles storing and updating of spacecraft
// position and attitude via internal and external updates.
// Internal updates are realized via attitude and position propagators.
// External updates can occur asynchronous,e.g. from orekit or RODOS sources.
//
module SatMobility like IMobility
{
    parameters:
        // signals...
        @signal[mobilityStateChanged];
        // submodule type parameters
        string attitudePropagatorType = default("AttitudePropagatorQuaternion");  // NED type of the attitude propagator
        string positionPropagatorType = default("PositionPropagatorKepler"); // NED type of the position propagator

        // Switch to enable/disable self-triggered updates of the satMobility module
        bool enableSelfTrigger = default(false);
        // Time interval for self-triggered updates, if enabled
        double selfTriggerTimeIv @unit(s) = default(10s);

        // Prefix of the singal name used for external updates
        // for this SatMobility module. The whole signal name is
        // then composed of "PREFIX<SatMobility Module Name>".
        string extUpdtSignalNamePrefix = default("extUpdate_");

        string initialRotationSpeed = default("0.00000001,0.00000002,-0.00000001"); // gives the satellite an initial angular velocity

        @class(SatMobility);
    submodules:
        attitudePropagator: <attitudePropagatorType> like IAttitudePropagator {
            parameters:
                @display("p=100,50");
        }
        positionPropagator: <positionPropagatorType> like IPositionPropagator {
            parameters:
                @display("p=100,125");
        }
}
