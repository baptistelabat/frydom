#include <frydom/core/math/FrVector.h>
#include "FrBoxShape.h"

#include "chrono/assets/ChBoxShape.h"

namespace frydom {
  FrBoxShape::FrBoxShape(double xSize, double ySize, double zSize) : m_box(std::make_shared<chrono::ChBoxShape>()) {
    m_box->GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
  }

  double FrBoxShape::xSize() const {
    return m_box->GetBoxGeometry().GetLengths().x();
  }

  double FrBoxShape::ySize() const {
    return m_box->GetBoxGeometry().GetLengths().y();
  }

  double FrBoxShape::zSize() const {
    return m_box->GetBoxGeometry().GetLengths().z();
  }

  std::shared_ptr<chrono::ChAsset> FrBoxShape::GetChronoAsset() {
    return m_box;
  }

  void FrBoxShape::Translate(const Direction &direction) const {
    m_box->Pos = internal::Vector3dToChVector(direction);
  }
}  // end namespace frydom
