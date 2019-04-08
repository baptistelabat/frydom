#include "frydom/shape/FrBoxShape.h"

#include "chrono/assets/ChBoxShape.h"

namespace frydom {
    FrBoxShape::FrBoxShape(double xSize, double ySize, double zSize) : m_box(std::make_shared<chrono::ChBoxShape>()) {
        m_box->GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
    }

    std::shared_ptr<chrono::ChAsset> FrBoxShape::GetChronoAsset() {
        return m_box;
    }
}  // end namespace frydom
