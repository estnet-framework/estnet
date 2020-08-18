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

package estnet.node.groundstation;

//
// Represents a ground label in the simulation
//
simple GroundLabel
{
    parameters:
        // 3D model parameters
        string modelURL;                      // the URL or filename of the 3D model to be used for the node
        double modelScale = default(1000000); // the scale of the 3D model
        // decorations and annotations
        string labelColor = default("#00ffffff");    // the color of the model label in hex RRGGBBAA format or "" to turn off labels
        double longitude @unit(deg);	// longitude at which the ground station is positioned on earth
        double latitude @unit(deg);		// latitude at which the ground station is positioned on earth
        string label = default("");		// label displayed in the view
        double timeStep @unit(s) = default(33ms);  // the time granularity of movement calculation (for 30fps)
        double startTime @unit(s) = default(0s);   // time when the movement starts
        @display("i=device/receiverdish;t=$label");
        @class(GroundLabel);
}
