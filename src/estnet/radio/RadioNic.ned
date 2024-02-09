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

package estnet.radio;

import inet.linklayer.contract.IMacProtocol;
import inet.linklayer.contract.IWirelessInterface;
import inet.physicallayer.contract.packetlevel.IRadio;
import inet.networklayer.common.InterfaceEntry;

@namespace(inet);

//
// Radio Network Adapter
//
module RadioNic extends InterfaceEntry like IWirelessInterface
{
    parameters:
        @display("i=block/ifcard;bgb=214,335;bgl=53");
        // so we don't need to define it for the mac and radio module separately
        double bitrate @unit(bps);
        string macType = default("IdealMacExtraStatistics");		// type of MAC layer protocol
        string radioType = default("UnitDiskRadio");	// type of Radio
        **.bitrate = default(bitrate);
        string interfaceTableModule = default(absPath(".interfaceTable")); // path to interface table module
        @class(InterfaceEntry);
        @nic;
    gates:
        input upperLayerIn;
        output upperLayerOut;
        input radioIn @labels(IdealRadioFrame);
    submodules:
        mac: <macType> like IMacProtocol {
            parameters:
                @display("p=98,207");
        }
        radio: <radioType> like IRadio {
            parameters:
                @display("p=98,278");
        }
    connections:
        upperLayerIn --> mac.upperLayerIn;
        mac.lowerLayerOut --> {  delay = 100ns; } --> radio.upperLayerIn;
        mac.upperLayerOut --> upperLayerOut;
        radioIn --> radio.radioIn;
        radio.upperLayerOut --> mac.lowerLayerIn;
}
