#include "frydom/shape/FrSphereShape.h"

#include "chrono/assets/ChSphereShape.h"

namespace frydom {

    FrSphereShape::FrSphereShape(double radius) : m_sphere(std::make_shared<chrono::ChSphereShape>()) {
        m_sphere->GetSphereGeometry().rad = radius;
    }
    double FrSphereShape::radius() const {
        return m_sphere->GetSphereGeometry().rad;
    }
    std::shared_ptr<chrono::ChAsset> FrSphereShape::GetChronoAsset() {
        return m_sphere;
    }

}  // end namespace frydom
