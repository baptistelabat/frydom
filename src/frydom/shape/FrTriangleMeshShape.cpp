#include "frydom/shape/FrTriangleMeshShape.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {
    FrTriangleMeshShape::FrTriangleMeshShape(
            std::shared_ptr<FrTriangleMeshConnected> mesh) : m_mesh(std::make_shared<chrono::ChTriangleMeshShape>()) {
        m_mesh->SetMesh(mesh);
    }
    std::shared_ptr<chrono::ChAsset> FrTriangleMeshShape::GetChronoAsset() {
        return m_mesh;
    }
}  // end namespace frydom
