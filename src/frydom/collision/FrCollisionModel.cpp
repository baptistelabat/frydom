//
// Created by lletourn on 21/05/19.
//

#include "FrCollisionModel.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {


        bool FrCollisionModel::AddSphere(double radius, const Position &pos) {

            auto chPos = Vector3dToChVector(pos);
            return ChModelBullet::AddSphere(radius, chPos);

        }

        bool
        FrCollisionModel::AddEllipsoid(double rx, double ry, double rz, const Position &pos, const FrRotation &rot) {

            auto chPos = Vector3dToChVector(pos);
            auto chRot = Fr2ChQuaternion(rot.GetQuaternion());

            return ChModelBullet::AddEllipsoid(rx, ry, rz, chPos, chRot);

        }

        bool FrCollisionModel::AddBox(double hx, double hy, double hz, const Position &pos, const FrRotation &rot) {

            auto chPos = Vector3dToChVector(pos);
            auto chRot = Fr2ChQuaternion(rot.GetQuaternion());

            return ChModelBullet::AddBox(hx, hy, hz, chPos, chRot);

        }

        bool
        FrCollisionModel::AddCylinder(double rx, double rz, double hy, const Position &pos, const FrRotation &rot) {

            auto chPos = Vector3dToChVector(pos);
            auto chRot = Fr2ChQuaternion(rot.GetQuaternion());

            return ChModelBullet::AddCylinder(rx, rz, hy, chPos, chRot);

        }

        bool FrCollisionModel::AddConvexHull(const std::vector<Position> &pointlist, const Position &pos,
                                             const FrRotation &rot) {

            std::vector<chrono::ChVector<double>> chVect;
            for (const auto &point : pointlist) {
                chVect.push_back(Vector3dToChVector(point));
            }

            auto chPos = Vector3dToChVector(pos);
            auto chRot = Fr2ChQuaternion(rot.GetQuaternion());

            return ChModelBullet::AddConvexHull(chVect, chPos, chRot);

        }

        bool FrCollisionModel::AddTriangleMesh(std::shared_ptr<FrTriangleMeshConnected> trimesh, bool is_static,
                                               bool is_convex, const Position &pos, const FrRotation &rot,
                                               double sphereswept_thickness) {

            auto chPos = Vector3dToChVector(pos);
            auto chRot = Fr2ChQuaternion(rot.GetQuaternion());

            return ChModelBullet::AddTriangleMesh(trimesh, is_static, is_convex, chPos, chRot, sphereswept_thickness);

        }

        bool
        FrCollisionModel::AddTriangleMeshConcave(std::shared_ptr<FrTriangleMeshConnected> trimesh, const Position &pos,
                                                 const FrRotation &rot) {

            auto chPos = Vector3dToChVector(pos);
            auto chRot = Fr2ChQuaternion(rot.GetQuaternion());

            return ChModelBullet::AddTriangleMeshConcave(trimesh, chPos, chRot);

        }
    } // end namespace frydom::internal

} // end namespace frydom