#include "frydom/shape/FrTriangleMeshShape.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {
  FrTriangleMeshShape::FrTriangleMeshShape(
          std::shared_ptr<FrTriangleMeshConnected> mesh) : chrono::ChTriangleMeshShape() {
      SetMesh(mesh);
  }
}  // end namespace frydom
