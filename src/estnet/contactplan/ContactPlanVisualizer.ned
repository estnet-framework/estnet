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

package estnet.contactplan;

//
// Visualizes lines of sight between nodes
// in the simulation.
//
simple ContactPlanVisualizer
{
    parameters:
        bool show = default(true);                            // whether to show the line of sights or not
        string satToSatColor = default("#00FF00BF");          // color of satellite to satellite line-of-sights
        string satToSatColorOnPlan = default("#FF00FABF");    // color of satellite to satellite line-of-sights that are in the current contact plan
        double satToSatWidth = default(2);                    // width satellite to satellite line-of-sights
        string satToGroundColor = default("#FFBF00BF");       // color of satellite to ground station line-of-sights
        string satToGroundColorOnPlan = default("#FF7200BF"); // color of satellite to satellite line-of-sights that are in the current contact plan
        double satToGroundWidth = default(2);                 // width of satellite to ground station line-of-sights
}
