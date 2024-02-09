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
// Creates OSG Nodes from the model
//
simple OsgEarthScene
{
    parameters:
        @class(OsgEarthScene);
        @display("i=misc/globe");
        string sceneModel;  					// 3D Earth Model, assuming project folder is in same parent folder as estnet
        string coordBaseColorX = default("");   // defines the color of the x-axis: #ff0000ff
        string coordBaseColorY = default("");   // defines the color of the y-axis: #00ff00ff
        string coordBaseColorZ = default("");   // defines the color of the z-axis: #0000ffff
        double coordBaseLength @unit(m) = default(1e7m);  // defines the length of the coordinate axis
        double coordBaseWidth @unit(m) = default(25m);  	// defines the width of the coordiate axis
}
