// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================

#include "FrNonlinearHydrostaticForce.h"

#include "frydom/core/body/FrBody.h"

#include "frydom/mesh/FrMesh.h"
#include "frydom/mesh/FrHydroMesh.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    FrNonlinearHydrostaticForce::FrNonlinearHydrostaticForce(const std::shared_ptr<FrHydroMesh> &HydroMesh) {
        m_hydro_mesh = HydroMesh;
    }


    void FrNonlinearHydrostaticForce::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce::Initialize();

    }

    void FrNonlinearHydrostaticForce::InitializeLog(){

        // This function initializes the logger for the nonlinear hydrostatic loads by giving the position of the center of buoyancy in the body frame.


        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CoBInWorld","m", fmt::format("Center of buoyancy in world reference frame in {}", c_logFrameConvention),
                 [this]() {return m_CoBInWorld;});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("OForceInWorld","N", fmt::format("Hydrostatic force, at CoB, in world reference frame in {}", c_logFrameConvention),
                 [this]() {return m_dummy;});

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInWorld","m", fmt::format("Center of buoyancy in world reference frame in {}", c_logFrameConvention),
                 [this]() {return GetCenterOfBuoyancyInWorld(c_logFrameConvention);});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInBody","m", fmt::format("Center of buoyancy in body reference frame in {}", c_logFrameConvention),
                 [this]() {return GetCenterOfBuoyancyInBody(c_logFrameConvention);});
//        m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                ("CenterOfBuoyancyInMesh","m", fmt::format("Center of buoyancy in mesh reference frame in {}", c_logFrameConvention),
//                 [this]() {return GetCenterOfBuoyancyInMesh(c_logFrameConvention);});

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("ForceInWorld","N", fmt::format("Hydrostatic force, at CoB, in world reference frame in {}", c_logFrameConvention),
                 [this]() {return GetHydrostaticForceInWorld(c_logFrameConvention);});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("ForceInBody","N", fmt::format("Hydrostatic force, at CoB, in body reference frame in {}", c_logFrameConvention),
                 [this]() {return GetHydrostaticForceInBody(c_logFrameConvention);});
//        m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                ("ForceInMesh","N", fmt::format("Hydrostatic force, at CoB, in mesh reference frame in {}", c_logFrameConvention),
//                 [this]() {return GetHydrostaticForceInMesh(c_logFrameConvention);});

        FrForce::InitializeLog();

    }

    void FrNonlinearHydrostaticForce::Compute(double time) {

        // This function computes the nonlinear hydrostatic loads.

        // Computation of the hydrostatic force.
        NonlinearHydrostatics NLhydrostatics(m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                             m_body->GetSystem()->GetGravityAcceleration()); // Creation of the NonlinearHydrostatics structure.

        // Compute the hydrostatic pressure on the clipped mesh
        NLhydrostatics.CalcPressureIntegration(m_hydro_mesh->GetClippedMesh());

        // Setting the nonlinear hydrostatic loads in world at the CoB in world.
        m_dummy = NLhydrostatics.GetNonlinearForce();

        // The translation of the body was not done for avoiding numerical errors.
        m_CoBInWorld = m_body->GetPosition(NWU) + NLhydrostatics.GetCenterOfBuoyancy();

        // The torque is computed from the hydrostatic force and the center of buoyancy.
        SetForceInWorldAtPointInWorld(m_dummy, m_CoBInWorld, NWU);

//        auto trucInMesh = GetHydrostaticForceInMesh(NWU);
        auto trucInBody = GetHydrostaticForceInBody(NWU);
        auto trucInWorld = GetHydrostaticForceInWorld(NWU);

//        auto COBInMesh = GetCenterOfBuoyancyInMesh(NWU);
        auto COBInBody = GetCenterOfBuoyancyInBody(NWU);
        auto COBInWorld = GetCenterOfBuoyancyInWorld(NWU);

        SetForceInWorldAtPointInWorld(GetHydrostaticForceInWorld(NWU), GetCenterOfBuoyancyInWorld(NWU), NWU);
//        SetForceInWorldAtPointInWorld(m_dummy, GetCenterOfBuoyancyInBody(NWU), NWU);
//        SetForceInWorldAtPointInWorld(GetHydrostaticForceInBody(NWU), GetCenterOfBuoyancyInBody(NWU), NWU);

    }

//    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInMesh(FRAME_CONVENTION fc) {
//
//        auto CoBInMesh = mesh::OpenMeshPointToVector3d<Position>(m_hydro_mesh->GetClippedMesh().GetCOG());
//
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInMesh);
//
//        return CoBInMesh;
//    }

    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){
        // This function returns the center of buoyancy of the clipped mesh in the bdy reference frame.

        return m_body->GetPointPositionInBody(GetCenterOfBuoyancyInWorld(fc),fc);

//        return m_hydro_mesh->GetMeshOffset().GetPosition(NWU) + m_hydro_mesh->GetMeshOffset().ProjectVectorFrameInParent(GetCenterOfBuoyancyInMesh(fc),fc);

    }

    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc) {

        auto CoBInWorld = mesh::OpenMeshPointToVector3d<Position>(m_hydro_mesh->GetClippedMesh().GetCOG());

        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;

        CoBInWorld += bodyPos;

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);

        return CoBInWorld;
    }

//    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInMesh(FRAME_CONVENTION fc) {
//        // This function performs the hydrostatic pressure integration.
//
//        Force hydrostaticForce = {0.,0.,0.};
//
//        // Loop over the faces.
//        for (mesh::FrMesh::FaceIter f_iter = m_hydro_mesh->GetClippedMesh().faces_begin(); f_iter != m_hydro_mesh->GetClippedMesh().faces_end(); ++f_iter) {
//
//            // Normal.
//            auto normal = m_hydro_mesh->GetClippedMesh().normal(*f_iter);
//
//            // Pressure*Area.
//            auto pressure = m_hydro_mesh->GetClippedMesh().data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z);
//
//            // Hydrostatic force without the term rho*g.
//            hydrostaticForce[0] += pressure*normal[0];
//            hydrostaticForce[1] += pressure*normal[1];
//            hydrostaticForce[2] += pressure*normal[2];
//
//        }
//
//        // Multiplication by rho*g
//        hydrostaticForce *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//
//        return hydrostaticForce;
//
//    }

    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInBody(FRAME_CONVENTION fc) {
        return m_body->ProjectVectorInBody(GetHydrostaticForceInWorld(fc),fc);
//        return m_hydro_mesh->GetMeshOffset().ProjectVectorFrameInParent(GetHydrostaticForceInMesh(fc),fc);
    }

    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInWorld(FRAME_CONVENTION fc) {
        // This function performs the hydrostatic pressure integration.

        Force hydrostaticForce = {0.,0.,0.};

        // Loop over the faces.
        for (mesh::FrMesh::FaceIter f_iter = m_hydro_mesh->GetClippedMesh().faces_begin(); f_iter != m_hydro_mesh->GetClippedMesh().faces_end(); ++f_iter) {

            // Normal.
            auto normal = m_hydro_mesh->GetClippedMesh().normal(*f_iter);

            // Pressure*Area.
            auto pressure = m_hydro_mesh->GetClippedMesh().data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z);

            // Hydrostatic force without the term rho*g.
            hydrostaticForce[0] += pressure*normal[0];
            hydrostaticForce[1] += pressure*normal[1];
            hydrostaticForce[2] += pressure*normal[2];

        }

        // Multiplication by rho*g
        hydrostaticForce *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

        return hydrostaticForce;
    }

    void FrNonlinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writing the clipped mesh in an output file.
//        m_clippedMesh.Write("Mesh_clipped_Hydrostatics.obj");
//        std::exit(0);

    }

    std::shared_ptr<FrNonlinearHydrostaticForce>
    make_nonlinear_hydrostatic_force(const std::shared_ptr<FrBody>& body, const std::shared_ptr<FrHydroMesh>& HydroMesh){

        // This function creates a (fully or weakly) nonlinear hydrostatic force object.

        // Construction of the (fully or weakly) nonlinear hydrostatic force object.
        auto forceHst = std::make_shared<FrNonlinearHydrostaticForce>(HydroMesh);

        // Add the (fully or weakly) nonlinear hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom