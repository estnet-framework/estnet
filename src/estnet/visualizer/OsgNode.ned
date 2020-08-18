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

package estnet.visualizer;

//
// Represents a Network-Node in the OsgScene (currently only groundstations and satellites)
// This module is only compatible with the OsgEarthScene and the SatMobility module as position source
//
simple OsgNode
{
    parameters:
        // OSG 3D model parameters
        string modelURL;											// 3D satellite icon path
        double modelScale = default(1);                             // Scaling factor for the 3D model
        string labelColor = default("#ffff00ff");                   // color of the model label
        double timeStep @unit(s) = default(60s);                	// temporal simulation resolution
        string orbitColor = default("#80C8C8C8");                   // color of the orbit circle
        double orbitResolution = default(100);                      // calculates 'orbitalResolution'
        double orbitUpdateFreq = default(10000);                    // determines how often the orbit is redrawn
        string satConeColor = default("#FFFFFF16");                 // defines the colors of satellite cones
        double satConeScaling = default(1);                         // defines the radius of satellite cones relative to a maximum line of sight cone
        string coordBaseColorX = default("");                       // defines the color of the x-axis: #ff0000ff
        string coordBaseColorY = default("");                       // defines the color of the y-axis: #00ff00ff
        string coordBaseColorZ = default("");                       // defines the color of the z-axis: #0000ffff
        double coordBaseLength = default(5);                        // defines the length of the coordinate axis
        double coordBaseWidth = default(25);                        // defines the width of the coordiate axis
}
