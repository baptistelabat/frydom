#include "frydom/shape/FrCylinderShape.h"

#include "chrono/assets/ChCylinderShape.h"

namespace frydom {
    FrCylinderShape::FrCylinderShape(double radius, double height) : m_cylinder(std::make_shared<chrono::ChCylinderShape>()) {
        m_cylinder->GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
        m_cylinder->GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
        m_cylinder->GetCylinderGeometry().rad = radius;
    }
    std::shared_ptr<chrono::ChAsset> FrCylinderShape::GetChronoAsset() {
      return m_cylinder;
    }
}  // end namespace frydom
