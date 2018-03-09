//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "frydom/hydrodynamics/FrHydroDB.h"
#include "FrHydroBody.h"

namespace frydom {

    void FrHydroBody::Set3DOF(const bool flag) {
        if (flag) {
            Set3DOF_ON();
        } else {
            Set3DOF_OFF();
        }
    }

    void FrHydroBody::Set3DOF_ON() {
        if (is3DOF){ return; }
        // TODO: voir si on peut pas faire quelque chose avec l'attribut flipped de ChLinkMate...

        try {
            if (!GetSystem()) {
                throw std::string("The body must be added to a system before the plane constraint is set");
            }
//            if (!GetSystem()->getFreeSurface()) {
//                throw std::string("A free surface has to be created before setting a plane constraint for a body");
//            }
        } catch(std::string const& msg) {
            std::cerr << msg << std::endl;
        }

        auto plane_constraint = std::make_shared<chrono::ChLinkMatePlane>();
        auto free_surface_body = GetSystem()->GetEnvironment()->GetFreeSurface()->GetBody();
        plane_constraint->Initialize(shared_from_this(), free_surface_body,
                                     true,
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(0, 0, 1),
                                     chrono::ChVector<>(0, 0, -1));
        system->AddLink(plane_constraint);
        constraint3DOF = plane_constraint;
        is3DOF = true;
    }

    void FrHydroBody::Set3DOF_OFF() {
        if (!is3DOF) { return; }

        system->RemoveLink(constraint3DOF);
        constraint3DOF->SetSystem(0);
        is3DOF = false;
    }

    void FrHydroBody::SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset) {

        m_hydro_mesh = mesh;

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

//        std::cout << "Updating body"  << std::endl;

        // Update heading
        auto euler_angles = quat_to_euler(GetRot(), CARDAN, RAD);
        m_heading = Normalize_0_2PI(euler_angles.z());

        // Update course
        auto body_velocity = GetVelocity(NWU);
        m_course = Normalize_0_2PI(atan2(body_velocity.y(), body_velocity.x()));

        // Update current relative velocity
        auto current_velocity = GetSystem()->GetEnvironment()->GetCurrent()->GetFluxVector(NWU);

        m_current_relative_velocity = body_velocity - current_velocity;

        // Update the current relative angle
        auto relative_velocity_angle = atan2(m_current_relative_velocity.y(),
                                             m_current_relative_velocity.x());
        m_current_relative_angle = Normalize__PI_PI(relative_velocity_angle - m_heading);

        // update parent class
        chrono::ChBodyAuxRef::Update(update_assets);

    }

    chrono::ChVector<double> FrHydroBody::GetCurrentRelativeVelocity(FrFrame frame) const{
        switch (frame) {  // TODO: avoir une fonction pour eviter la recopie systematique...
            case NWU:
                return m_current_relative_velocity;
            case NED:
                return NWU2NED(m_current_relative_velocity);
        }
    }

    chrono::ChVector<> FrHydroBody::GetCurrentRelativeVelocity(const chrono::ChVector<>& localpos,
                                                               FrFrame frame) const {
        auto current_velocity = GetSystem()->GetEnvironment()->GetCurrent()->GetFluxVector(NWU);
        auto velocity = PointSpeedLocalToParent(localpos);
        auto current_relative_velocity = current_velocity - velocity;
        switch (frame) {
            case NWU:
                return current_relative_velocity;
            case NED:
                return NWU2NED(current_relative_velocity);
        }
    }

    void FrHydroBody::SetNEDHeading(const double heading_angle, ANGLE_UNIT angleUnit) {
        auto quaternion = euler_to_quat(0., 0., -heading_angle, CARDAN, angleUnit);
        SetRot(quaternion);
    }

    void FrHydroBody::SetNEDHeading(const chrono::ChVector<>& unit_vector) {
        // TODO: verifier qu'on a un vecteur unite
        auto heading_angle = atan2(unit_vector.y(), unit_vector.x());
        FrHydroBody::SetNEDHeading(heading_angle, RAD);
    }

    double FrHydroBody::GetTransverseUnderWaterArea() const {
        return m_transverse_area;
    }

    void FrHydroBody::SetTransverseUnderWaterArea(double transverse_area) {
        FrHydroBody::m_transverse_area = transverse_area;
    }

    double FrHydroBody::GetLateralUnderWaterArea() const {
        return m_lateral_area;
    }

    void FrHydroBody::SetLateralUnderWaterArea(double lateral_area) {
        FrHydroBody::m_lateral_area = lateral_area;
    }

    double FrHydroBody::GetLpp() const {
        return m_length_between_perpendicular;
    }

    void FrHydroBody::SetLpp(double lpp) {
        FrHydroBody::m_length_between_perpendicular = lpp;
    }

    double FrHydroBody::GetWettedSurface() const {
        return m_wetted_surface;
    }

    void FrHydroBody::SetWettedSurface(double wetted_surface) {
        FrHydroBody::m_wetted_surface = wetted_surface;
    }

//    void FrHydroBody::SetBEMBody(std::shared_ptr<FrBEMBody> BEMBody) {
//        m_BEMBody = BEMBody;
//        BEMBody->SetHydroBody(this);
//
//        // TODO: Il faut renseigner les masses ajoutees a l'infini !!!!
//
//    }

    void FrHydroBody::SetCurrentRefFrameAsEquilibrium() {
        auto freeSurfaceFrame = dynamic_cast<FrOffshoreSystem*>(system)->GetEnvironment()->GetFreeSurface()->GetFrame();
        chrono::ChFrame<double> eqFrame0 = GetFrame_REF_to_abs() >> freeSurfaceFrame->GetInverse();
        SetEquilibriumFrame(eqFrame0);
    }

    chrono::ChFrame<double> FrHydroBody::GetEquilibriumFrame() const {
        auto freeSurfaceFrame = dynamic_cast<FrOffshoreSystem*>(system)->GetEnvironment()->GetFreeSurface()->GetFrame();
        auto eqFrame = m_equilibriumFrame >> freeSurfaceFrame->GetInverse();
        return eqFrame;
    }

}  // end namespace frydom
