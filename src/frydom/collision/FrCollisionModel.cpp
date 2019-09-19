//
// Created by lletourn on 21/05/19.
//

#include "FrCollisionModel.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {
        template<typename OffshoreSystemType>
        FrCollisionModelBase<OffshoreSystemType>::FrCollisionModelBase(FrCollisionModel<OffshoreSystemType> *collisionModel) :
            m_frydomCollisionModel(collisionModel) {

        }

    } // end namespace frydom::internal

    template<typename OffshoreSystemType>
    FrCollisionModel<OffshoreSystemType>::FrCollisionModel() {
      m_chronoCollisionModel = std::make_shared<internal::FrCollisionModelBase>(this);
      m_chronoCollisionModel->ClearModel();
    }

    template<typename OffshoreSystemType>
    bool FrCollisionModel<OffshoreSystemType>::AddSphere(double radius, const Position &pos) {

      auto chPos = internal::Vector3dToChVector(pos);
      return m_chronoCollisionModel->AddSphere(radius, chPos);

    }

    template<typename OffshoreSystemType>
    bool
    FrCollisionModel<OffshoreSystemType>::AddEllipsoid(double rx, double ry, double rz, const Position &pos, const FrRotation &rot) {

      auto chPos = internal::Vector3dToChVector(pos);
      auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

      return m_chronoCollisionModel->AddEllipsoid(rx, ry, rz, chPos, chRot);

    }

    template<typename OffshoreSystemType>
    bool FrCollisionModel<OffshoreSystemType>::AddBox(double hx, double hy, double hz, const Position &pos, const FrRotation &rot) {

      auto chPos = internal::Vector3dToChVector(pos);
      auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

      return m_chronoCollisionModel->AddBox(hx, hy, hz, chPos, chRot);

    }

    template<typename OffshoreSystemType>
    bool
    FrCollisionModel<OffshoreSystemType>::AddCylinder(double rx, double rz, double hy, const Position &pos, const FrRotation &rot) {

      auto chPos = internal::Vector3dToChVector(pos);
      auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

      return m_chronoCollisionModel->AddCylinder(rx, rz, hy, chPos, chRot);

    }

    template<typename OffshoreSystemType>
    bool FrCollisionModel<OffshoreSystemType>::AddConvexHull(const std::vector<Position> &pointlist, const Position &pos,
                                         const FrRotation &rot) {

      std::vector<chrono::ChVector<double>> chVect;
      for (const auto &point : pointlist) {
        chVect.push_back(internal::Vector3dToChVector(point));
      }

      auto chPos = internal::Vector3dToChVector(pos);
      auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

      return m_chronoCollisionModel->AddConvexHull(chVect, chPos, chRot);

    }

    template<typename OffshoreSystemType>
    bool FrCollisionModel<OffshoreSystemType>::AddTriangleMesh(std::shared_ptr<FrTriangleMeshConnected> trimesh, bool is_static,
                                           bool is_convex, const Position &pos, const FrRotation &rot,
                                           double sphereswept_thickness) {

      auto chPos = internal::Vector3dToChVector(pos);
      auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

      return m_chronoCollisionModel->AddTriangleMesh(trimesh, is_static, is_convex, chPos, chRot,
                                                     sphereswept_thickness);

    }

    template<typename OffshoreSystemType>
    bool
    FrCollisionModel<OffshoreSystemType>::AddTriangleMeshConcave(std::shared_ptr<FrTriangleMeshConnected> trimesh, const Position &pos,
                                             const FrRotation &rot) {

      auto chPos = internal::Vector3dToChVector(pos);
      auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

      return m_chronoCollisionModel->AddTriangleMeshConcave(trimesh, chPos, chRot);

    }

    template<typename OffshoreSystemType>
    void FrCollisionModel<OffshoreSystemType>::Initialize() {
      m_chronoCollisionModel->BuildModel();
    }

    template<typename OffshoreSystemType>
    void FrCollisionModel<OffshoreSystemType>::ClearModel() {
      m_chronoCollisionModel->ClearModel();
    }

    template<typename OffshoreSystemType>
    void FrCollisionModel<OffshoreSystemType>::BuildModel() {
      m_chronoCollisionModel->BuildModel();
    }

    template<typename OffshoreSystemType>
    bool FrCollisionModel<OffshoreSystemType>::AddTriangleMesh(const std::string &obj_filename, const Position &pos, const FrRotation &rot,
                                           bool is_static, bool is_convex, double sphereswept_thickness) {

      auto mesh = std::make_shared<FrTriangleMeshConnected>();
      mesh->LoadWavefrontMesh(obj_filename);

      return AddTriangleMesh(mesh, is_static, is_convex, pos, rot, sphereswept_thickness);

    }

} // end namespace frydom
