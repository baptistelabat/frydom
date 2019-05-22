//
// Created by lletourn on 21/05/19.
//

#include "FrCollisionModel.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {
        FrCollisionModelBase::FrCollisionModelBase(FrCollisionModel* collisionModel) :
        m_frydomCollisionModel(collisionModel) {

        }
        
    } // end namespace frydom::internal


    FrCollisionModel::FrCollisionModel(){
        m_chronoCollisionModel = std::make_shared<internal::FrCollisionModelBase>(this);
        m_chronoCollisionModel->ClearModel();
    }

    bool FrCollisionModel::AddSphere(double radius, const Position &pos) {

        auto chPos = internal::Vector3dToChVector(pos);
        return m_chronoCollisionModel->AddSphere(radius, chPos);

    }

    bool
    FrCollisionModel::AddEllipsoid(double rx, double ry, double rz, const Position &pos, const FrRotation &rot) {

        auto chPos = internal::Vector3dToChVector(pos);
        auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

        return m_chronoCollisionModel->AddEllipsoid(rx, ry, rz, chPos, chRot);

    }

    bool FrCollisionModel::AddBox(double hx, double hy, double hz, const Position &pos, const FrRotation &rot) {

        auto chPos = internal::Vector3dToChVector(pos);
        auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

        return m_chronoCollisionModel->AddBox(hx, hy, hz, chPos, chRot);

    }

    bool
    FrCollisionModel::AddCylinder(double rx, double rz, double hy, const Position &pos, const FrRotation &rot) {

        auto chPos = internal::Vector3dToChVector(pos);
        auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

        return m_chronoCollisionModel->AddCylinder(rx, rz, hy, chPos, chRot);

    }

    bool FrCollisionModel::AddConvexHull(const std::vector<Position> &pointlist, const Position &pos,
                                         const FrRotation &rot) {

        std::vector<chrono::ChVector<double>> chVect;
        for (const auto &point : pointlist) {
            chVect.push_back(internal::Vector3dToChVector(point));
        }

        auto chPos = internal::Vector3dToChVector(pos);
        auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

        return m_chronoCollisionModel->AddConvexHull(chVect, chPos, chRot);

    }

    bool FrCollisionModel::AddTriangleMesh(std::shared_ptr<FrTriangleMeshConnected> trimesh, bool is_static,
                                           bool is_convex, const Position &pos, const FrRotation &rot,
                                           double sphereswept_thickness) {

        auto chPos = internal::Vector3dToChVector(pos);
        auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

        return m_chronoCollisionModel->AddTriangleMesh(trimesh, is_static, is_convex, chPos, chRot, sphereswept_thickness);

    }

    bool
    FrCollisionModel::AddTriangleMeshConcave(std::shared_ptr<FrTriangleMeshConnected> trimesh, const Position &pos,
                                             const FrRotation &rot) {

        auto chPos = internal::Vector3dToChVector(pos);
        auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());

        return m_chronoCollisionModel->AddTriangleMeshConcave(trimesh, chPos, chRot);

    }

    void FrCollisionModel::Initialize() {
        m_chronoCollisionModel->BuildModel();
    }

    void FrCollisionModel::ClearModel() {
        m_chronoCollisionModel->ClearModel();
    }

    void FrCollisionModel::BuildModel() {
        m_chronoCollisionModel->BuildModel();
    }

    bool FrCollisionModel::AddTriangleMesh(const std::string &obj_filename, const Position &pos, const FrRotation &rot,
                                           bool is_static, bool is_convex, double sphereswept_thickness) {

        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);

        return AddTriangleMesh(mesh,is_static,is_convex,pos,rot,sphereswept_thickness);

    }

} // end namespace frydom