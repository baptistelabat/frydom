#include "frydom/shape/FrCylinderShape.h"

namespace frydom {

FrCylinderShape::FrCylinderShape(double radius, double height) : chrono::ChCylinderShape() {
    GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
    GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
    GetCylinderGeometry().rad = radius;
}

}  // end namespace frydom
