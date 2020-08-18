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

#ifdef WITH_OSG

#include "OsgUtils.h"

#include <osg/Geometry>
#include <osgEarthSymbology/Color>

namespace estnet {

osg::ref_ptr<osg::Drawable> createLineBetweenPoints(osg::Vec3 start,
                                                    osg::Vec3 end, float width,
                                                    osg::Vec4 color) {
  osg::ref_ptr<osg::Geometry> orbitGeom = new osg::Geometry;
  osg::ref_ptr<osg::DrawArrays> drawArrayLines =
      new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
  osg::ref_ptr<osg::Vec3Array> vertexData = new osg::Vec3Array;

  orbitGeom->addPrimitiveSet(drawArrayLines);
  auto stateSet = orbitGeom->getOrCreateStateSet();
  stateSet->setAttributeAndModes(new osg::LineWidth(width),
                                 osg::StateAttribute::ON |
                                     osg::StateAttribute::OVERRIDE);
  stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  auto depth = new osg::Depth;
  depth->setWriteMask(false);
  stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

  vertexData->push_back(start);
  vertexData->push_back(end);

  orbitGeom->setVertexArray(vertexData);
  drawArrayLines->setFirst(0);
  drawArrayLines->setCount(vertexData->size());

  osg::ref_ptr<osg::Vec4Array> colorData = new osg::Vec4Array;
  colorData->push_back(osgEarth::Color(color));
  orbitGeom->setColorArray(colorData, osg::Array::BIND_OVERALL);
  return orbitGeom;
}

osg::ref_ptr<osg::Geode> drawCoordinateSystem(osg::ref_ptr<osg::Geode> group,
                                              const std::string& coordBaseColorX,
                                              const std::string& coordBaseColorY,
                                              const std::string& coordBaseColorZ,
                                              double coordBaseLength,
                                              double coordBaseWidth) {
  osg::Vec3d startVector(0, 0, 0);
  osg::Vec3d xVector(coordBaseLength, 0, 0);
  osg::Vec3d yVector(0, coordBaseLength, 0);
  osg::Vec3d zVector(0, 0, coordBaseLength);

  // line representing the x-axis
  if (!coordBaseColorX.empty()) {
    osg::ref_ptr<osg::Drawable> lineX = createLineBetweenPoints(
        startVector, xVector, coordBaseWidth, osgEarth::Color(coordBaseColorX));
    group->addDrawable(lineX.get());
  }
  // line representing the y-axis
  if (!coordBaseColorY.empty()) {
    osg::ref_ptr<osg::Drawable> lineY = createLineBetweenPoints(
        startVector, yVector, coordBaseWidth, osgEarth::Color(coordBaseColorY));
    group->addDrawable(lineY.get());
  }
  // line representing the z-axis
  if (!coordBaseColorZ.empty()) {
    osg::ref_ptr<osg::Drawable> lineZ = createLineBetweenPoints(
        startVector, zVector, coordBaseWidth, osgEarth::Color(coordBaseColorZ));
    group->addDrawable(lineZ.get());
  }
  return group;
}

}  // namespace estnet

#endif
