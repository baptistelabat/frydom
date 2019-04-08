#include "frydom/shape/FrCylinderShape.h"

#include "chrono/assets/ChCylinderShape.h"

namespace frydom {
    FrCylinderShape::FrCylinderShape(double radius, double height) : m_cylinder(std::make_shared<chrono::ChCylinderShape>()) {
        m_cylinder->GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
        m_cylinder->GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
        m_cylinder->GetCylinderGeometry().rad = radius;
    }
    double FrCylinderShape::radius() const {
        return m_cylinder->GetCylinderGeometry().rad;
    }
    double FrCylinderShape::height() const {
        return fabs(m_cylinder->GetCylinderGeometry().p2.y() - m_cylinder->GetCylinderGeometry().p1.y());
    }
    std::shared_ptr<chrono::ChAsset> FrCylinderShape::GetChronoAsset() {
      return m_cylinder;
    }
}  // end namespace frydom
