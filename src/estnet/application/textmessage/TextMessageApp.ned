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

package estnet.application.textmessage;

import estnet.application.base.BasicApp;

//
//  App that sends a given string as packet payload
//  The received text message is compared to a expected message
//  a mulitplier can be used to increase the packet size by mulitples
//  of the given string
//
simple TextMessageApp extends BasicApp
{
    parameters:
        string textMessage = default("<Packet Content>"); 	// content of packets sent by app
        string expectedMessage = default(textMessage); 		// expected content of received packets
        int multiplier = default(1);						// multiples the length of the text message
        @class(TextMessageApp);
}
