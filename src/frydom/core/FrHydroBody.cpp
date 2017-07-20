//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBodyAuxRef.h"
#include <chrono/assets/ChTriangleMeshShape.h>

#include "FrHydroBody.h"
#include "FrOffshoreSystem.h"


namespace frydom {


    void FrHydroBody::SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset) {

        hydro_mesh = mesh;

        if (as_asset) {
            // Registering the mesh as an asset
            auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
            shape->SetMesh(*mesh);
            AddAsset(shape);
        }

        // TODO: Ajouter automatiquement un clipper
    }

    void FrHydroBody::SetHydroMesh(std::string obj_filename, bool as_asset) {
        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        SetHydroMesh(mesh, as_asset);
    }

    void FrHydroBody::Update(bool update_assets) {

        std::cout << "Updating body"  << std::endl;

        // Update heading
        auto euler_angles = quat_to_euler(GetRot(), CARDAN, RAD);
        m_heading = euler_angles.z();

        // Update course
        auto body_velocity = GetVelocity(NWU);
        m_course = atan2(body_velocity.y(), body_velocity.x());

//        // Update slideslip
//        m_sideslip = m_course - m_heading;

        // Update current relative velocity
        auto mysystem = dynamic_cast<FrOffshoreSystem*>(system);
        auto current_velocity = mysystem->GetCurrent()->GetFluxVector(NWU);
        m_current_relative_velocity = body_velocity - current_velocity;

        // Update the current relative angle
        auto relative_velocity_angle = atan2(m_current_relative_velocity.y(),
                                             m_current_relative_velocity.x());
        m_current_relative_angle = relative_velocity_angle - m_heading;

        // update parent class
        chrono::ChBodyAuxRef::Update(update_assets);
    }

    chrono::ChVector<> FrHydroBody::GetCurrentRelativeVelocity(FrFrame frame) {
        switch (frame) {  // TODO: avoir une fonction pour eviter la recopie systematique...
            case NWU:
                return m_current_relative_velocity;
            case NED:
                return NWU2NED(m_current_relative_velocity);
        }
    }

    void FrHydroBody::SetNEDHeading(const double heading_angle, FrAngleUnit angleUnit) {
        auto quaternion = euler_to_quat(0., 0., -heading_angle, CARDAN, angleUnit);
        SetRot(quaternion);
    }


}  // end namespace frydom