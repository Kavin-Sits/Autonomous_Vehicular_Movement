#include <algorithm>
#include <vector>

#include "math/geometry.h"
#include "math/math_util.h"
#include "math/shapes.h"

using geometry::Line2f;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;

Circle::Circle(Eigen::Vector2f origin, float radiusVal){
    center = origin;
    radius = radiusVal;
}

bool Circle::containsPoint(Eigen::Vector2f point)
{
    return ((point - center).squaredNorm() <= radius * radius);
}
 
bool Circle::intersectsLine(Line2f line){
    if((line.p0-center).squaredNorm() <= radius * radius || (line.p1-center).squaredNorm() <= radius * radius){
        return true;
    }

    Vector2f ab = line.p1 - line.p0;
    Vector2f ac = center - line.p0;
    float t = ac.dot(ab) / ab.squaredNorm();
    t = std::max(std::min(t, (float)1.0), (float)0);

    Vector2f p = line.p0 + t * ab;//ab.norm();

    return (p-center).squaredNorm() <= radius * radius;
}

vector<Vector2f> Circle::intersectionPt(Line2f line) {
    Vector2f p0 = line.p0;
    Vector2f p1 = line.p1;
    Vector2f d = p1 - p0;
    Vector2f f = p0 - center;

    float a = d.dot(d);
    float b = 2 * f.dot(d);
    float c = f.dot(f) - radius * radius;

    float discriminant = b * b - 4 * a * c;
    vector<Vector2f> intersections;

    if (discriminant >= 0) {
        // line might intersect the circle; discriminant = 0 means the line is tangent
        discriminant = std::sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        if (t1 >= 0 && t1 <= 1) {
            intersections.push_back(p0 + d * t1);
        }
        if (t2 >= 0 && t2 <= 1 && discriminant > 0) { // discriminant > 0 checks for two distinct points
            intersections.push_back(p0 + d * t2);
        }
    }

    return intersections;
}

Rectangle::Rectangle(vector<Vector2f> points){
    for(int i=0; i<4; i++){
        int j = (i+1) % 4;
        sides.push_back(Line2f(points[i], points[j]));
    }
}

bool Rectangle::containsPoint(Eigen::Vector2f point){
    Line2f tryLine = Line2f(INTMAX_MAX, INTMAX_MAX, point.x(), point.y());
    Line2f tryLine2 = Line2f(-INTMAX_MAX, -INTMAX_MAX, point.x(), point.y());
    return intersectsLine(tryLine) && intersectsLine(tryLine2);
}

bool Rectangle::intersectsLine(Line2f line){
    for(int i=0; i<(int)sides.size(); i++){
        if (sides[i].Intersects(line)){
            return true;
        }
    }
    return false;
}
