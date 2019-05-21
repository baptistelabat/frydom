#include "FrTriangleMeshShape.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {
    namespace internal {
        /// Help to convert chrono vector to Eigen::Matrix<type, 3, 1>
        template <typename T>
        Eigen::Matrix<T, 3, 1> MakeEigenVector(const chrono::ChVector<T>& vector) {
            Eigen::Matrix<T, 3, 1> out;
            out << vector.x(),
                   vector.y(),
                   vector.z();
            return out;
        }
    }

    FrTriangleMeshShape::FrTriangleMeshShape(
            std::shared_ptr<FrTriangleMeshConnected> mesh) : m_mesh(std::make_shared<chrono::ChTriangleMeshShape>()) {
        m_mesh->SetMesh(mesh);
    }
    std::shared_ptr<chrono::ChAsset> FrTriangleMeshShape::GetChronoAsset() {
        return m_mesh;
    }
    std::vector<FrTriangleMeshShape::Vertex> FrTriangleMeshShape::vertices() const {
        std::vector<FrTriangleMeshShape::Vertex> result;
        for (const auto& vertex : m_mesh->GetMesh()->m_vertices) {
            result.push_back(internal::MakeEigenVector(vertex));
        }
        return result;
    }
    std::vector<FrTriangleMeshShape::Normal> FrTriangleMeshShape::normals() const {
        std::vector<FrTriangleMeshShape::Normal> result;
        for (const auto& normal : m_mesh->GetMesh()->m_normals) {
            result.push_back(internal::MakeEigenVector(normal));
        }
        return result;
    }
    std::vector<FrTriangleMeshShape::FaceVertexIndex> FrTriangleMeshShape::faceVertexIndices() const {
        std::vector<FrTriangleMeshShape::FaceVertexIndex> result;
        for (const auto& index : m_mesh->GetMesh()->m_face_v_indices) {
            result.push_back(internal::MakeEigenVector(index));
        }
        return result;
    }
    std::vector<FrTriangleMeshShape::FaceNormalIndex> FrTriangleMeshShape::faceNormalIndices() const {
        std::vector<FrTriangleMeshShape::FaceNormalIndex> result;
        for (const auto& index : m_mesh->GetMesh()->m_face_n_indices) {
            result.push_back(internal::MakeEigenVector(index));
        }
        return result;
    }
}  // end namespace frydom
