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

#ifndef __OSG_OSG_UTILS__
#define __OSG_OSG_UTILS__

#ifdef WITH_OSG
#include <osg/Depth>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Vec3>
#include <osg/Vec4>

namespace estnet {

/**
 * @brief draws a line between start and end of the given width and colors.
 */
osg::ref_ptr<osg::Drawable> createLineBetweenPoints(osg::Vec3 start,
                                                    osg::Vec3 end, float width,
                                                    osg::Vec4 color);

/**
 * @brief returns a geode to draw a coordinate frame at any position and attitude
 */
osg::ref_ptr<osg::Geode> drawCoordinateSystem(osg::ref_ptr<osg::Geode> group,
                                              const std::string& coordBaseColorX,
                                              const std::string& coordBaseColorY,
                                              const std::string& coordBaseColorZ,
                                              double coordBaseLength,
                                              double coordBaseWidth);

}  // namespace estnet

#endif

#endif
