#include "frydom/shape/FrBoxShape.h"

#include "chrono/assets/ChBoxShape.h"
#include "chrono/serialization/ChArchive.h"

#include "frydom/shape/FrShapeFactory.h"

namespace frydom {
    FrBoxShape::FrBoxShape(double xSize, double ySize, double zSize) : chrono::ChBoxShape() {
      GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
    }
}  // end namespace frydom
