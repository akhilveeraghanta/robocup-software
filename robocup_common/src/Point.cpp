#include "Geometry2d/Point.hpp"
#include "Geometry2d/Util.hpp"

namespace Geometry2d {
bool Point::nearlyEquals(Point other) const {
    return nearlyEqual(x(), other.x()) && nearlyEqual(y(), other.y());
}
}
