#include "frydom/shape/FrBoxShape.h"

namespace frydom {
    FrBoxShape::FrBoxShape(double xSize, double ySize, double zSize) : chrono::ChBoxShape() {
      GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
    }
}  // end namespace frydom
