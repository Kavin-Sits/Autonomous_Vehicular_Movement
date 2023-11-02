//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    line2d.h
\brief   2D Line Segment representation
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>


#include "math/geometry.h"
#include "math/math_util.h"
#include "math/line2d.h"

using geometry::Line2f;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;

class Circle{
    public:
    Vector2f center;
    float radius;

    Circle(Eigen::Vector2f origin, float radius);
    bool containsPoint(Eigen::Vector2f point);
    bool intersectsLine(Line2f line);
};

class Rectangle{
    public:
    vector<Line2f> sides;

    Rectangle(vector<Vector2f> points);
    bool containsPoint(Eigen::Vector2f point);
    bool intersectsLine(Line2f line);
};