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

package estnet.application.contract;

import estnet.application.contract.IApp;

//
// Interface for data generating apps
//
module AppWrapper
{
    parameters:
        string appType = default("BasicApp");
        int nodeNo;		// Node id
        int appId;
    gates:
        input wrapperIn;
        output wrapperOut;
    submodules:
    	app: <appType> like IApp {
    		parameters:
    			nodeNo = nodeNo;
                appId = appId;
    	}
    connections:
    	wrapperIn --> app.appIn;
    	wrapperOut <-- app.appOut;

}