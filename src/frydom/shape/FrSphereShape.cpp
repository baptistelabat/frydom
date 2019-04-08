#include "frydom/shape/FrSphereShape.h"

namespace frydom {

    FrSphereShape::FrSphereShape(double radius) : chrono::ChSphereShape() {
        GetSphereGeometry().rad = radius;
    }

}  // end namespace frydom
