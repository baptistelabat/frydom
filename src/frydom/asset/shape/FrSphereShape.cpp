#include "FrSphereShape.h"

#include "chrono/assets/ChSphereShape.h"

namespace frydom {

  FrSphereShape::FrSphereShape(double radius, const Position& relative_position, FRAME_CONVENTION fc) :
  m_sphere(std::make_shared<chrono::ChSphereShape>()) {
    m_sphere->GetSphereGeometry().rad = radius;

    Position pos = relative_position;
    if (IsNED(fc)) {
      internal::SwapFrameConvention<Position>(pos);
    }
    m_sphere->GetSphereGeometry().center = internal::Vector3dToChVector(pos);
  }

  double FrSphereShape::radius() const {
    return m_sphere->GetSphereGeometry().rad;
  }

  std::shared_ptr<chrono::ChAsset> FrSphereShape::GetChronoAsset() {
    return m_sphere;
  }

}  // end namespace frydom
