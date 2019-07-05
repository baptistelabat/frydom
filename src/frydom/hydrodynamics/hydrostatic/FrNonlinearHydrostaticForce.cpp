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

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    FrNonlinearHydrostaticForce::FrNonlinearHydrostaticForce(const std::shared_ptr<FrHydroMesh> &HydroMesh) {
        m_hydroMesh = HydroMesh;
    }

    void FrNonlinearHydrostaticForce::AddFields(){

        FrForce::AddFields();

        // This function initializes the logger for the nonlinear hydrostatic loads by giving the position of the center of buoyancy in the body frame.

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInBody","m", fmt::format("Center of buoyancy in body reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetCenterOfBuoyancyInBody(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInWorld","m", fmt::format("Center of buoyancy in world reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetCenterOfBuoyancyInWorld(GetLogFrameConvention());});

//        m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                ("ForceInWorld","N", fmt::format("Hydrostatic force, at CoB, in world reference frame in {}", GetLogFrameConvention()),
//                 [this]() {return GetHydrostaticForceInWorld(GetLogFrameConvention());});
//        m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                ("ForceInBody","N", fmt::format("Hydrostatic force, at CoB, in body reference frame in {}", GetLogFrameConvention()),
//                 [this]() {return GetHydrostaticForceInBody(GetLogFrameConvention());});

    }

    void FrNonlinearHydrostaticForce::Compute(double time) {

        // This function computes the nonlinear hydrostatic loads.
//        SetForceTorqueInWorldAtPointInBody(GetHydrostaticForceInWorld(NWU), GetHydrostaticTorqueInWorld(NWU), Position(), NWU);

        SetForceInWorldAtPointInWorld(GetHydrostaticForceInWorld(NWU), GetCenterOfBuoyancyInWorld(NWU), NWU);

    }


    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){
        return m_body->GetPointPositionInBody(GetCenterOfBuoyancyInWorld(fc),fc);
    }

    // Not working for clipped mesh with wave plane
    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc) {

        auto Fh = GetHydrostaticForceInWorld(fc).GetFz();

        auto Mh = GetHydrostaticTorqueInWorld(fc);


        Torque Ms = {0., 0., 0.};

        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());

        // Loop over the faces.
        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {

            // Normal.
            auto normal = clippedMesh->normal(*f_iter);

            double xx = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
            double xy = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XY);
            double xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);

            // Hydrostatic torque without the term rho*g.
            Ms[0] += xy*normal[2] - xz*normal[1];
            Ms[1] += xz*normal[0] - xx*normal[2];
            Ms[2] += xx*normal[1] - xy*normal[0];

        }

        // Multiplication by rho*g
        Ms *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

//        assert(- Ms.GetMz() - Mh.GetMx() < 1E-5);

        Position Test(-Mh.GetMy() / Fh, Mh.GetMx() / Fh, -Ms.GetMy() / Fh);
        //--------------------------------------------------------------------------------------------------------------


        Position COB = {0,0,0};
        // Loop over the faces.
        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {

            // Normal.
            auto normal = clippedMesh->normal(*f_iter);

            double x2 = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
            double y2 = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2);
            double z2 = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);

            COB += (x2 + y2 +z2) * Position(normal[0],normal[1],normal[2]);

        }

        auto volume = clippedMesh->GetVolume();
        COB /= 2*volume;
        
        
        //--------------------------------------------------------------------------------------------------------------

        // clipped mesh is expressed in the world reference frame, but its horizontal position is centered around (0.,0.)
        auto CoBInWorld = m_hydroMesh->GetClippedMesh().GetCOG();

        // Addition of the horizontal position of the body
        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;
        CoBInWorld += bodyPos;

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);

//        return {-Mh.GetMy() / Fh, Mh.GetMx() / Fh, Ms.GetMy() / Fh};

        return CoBInWorld;
    }
//    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc) {
//
//        // clipped mesh is expressed in the world reference frame, but its horizontal position is centered around (0.,0.)
//        auto CoBInWorld = m_hydroMesh->GetClippedMesh().GetCOG();
//
//        // Addition of the horizontal position of the body
//        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;
//        CoBInWorld += bodyPos;
//
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);
//
//        return CoBInWorld;
//    }

    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInBody(FRAME_CONVENTION fc) {
        return m_body->ProjectVectorInBody(GetHydrostaticForceInWorld(fc),fc);
    }

    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInWorld(FRAME_CONVENTION fc) {
        // This function performs the hydrostatic pressure integration.

        Force hydrostaticForce = {0.,0.,0.};
        
        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());

        // Loop over the faces.
        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {

            // Normal.
            auto normal = clippedMesh->normal(*f_iter);

            // Pressure*Area.
            auto pressure = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z);

            // Hydrostatic force without the term rho*g.
            hydrostaticForce[0] += pressure*normal[0];
            hydrostaticForce[1] += pressure*normal[1];
            hydrostaticForce[2] += pressure*normal[2];

        }

        // Multiplication by rho*g
        hydrostaticForce *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(hydrostaticForce);

        return hydrostaticForce;
    }

    Torque FrNonlinearHydrostaticForce::GetHydrostaticTorqueInWorld(FRAME_CONVENTION fc) {

        Torque Ms = {0., 0., 0.};

        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
//        clippedMesh->Write("TorqueMesh.obj");

        // Loop over the faces.
        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {

            // Normal.
            auto normal = clippedMesh->normal(*f_iter);

//            double xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
//            double yz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2);
//            double zz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
            double xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);
            double yz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_YZ);
            double zz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);

//            Position X(xz, yz, zz);
//            Position N(normal[0],normal[1],normal[2]);
//
//            Ms += X.cwiseProduct(N);

            // Hydrostatic torque without the term rho*g.
            Ms[0] += yz*normal[2] - zz*normal[1];
            Ms[1] += zz*normal[0] - xz*normal[2];
            Ms[2] += xz*normal[1] - yz*normal[0];

        }

        auto volume = clippedMesh->GetVolume();
//        Ms /= 2*volume;

        auto COG = clippedMesh->GetCOG();

        // Multiplication by rho*g
        Ms *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(Ms);

        return Ms;

    }

    Torque FrNonlinearHydrostaticForce::GetHydrostaticTorqueInBody(FRAME_CONVENTION fc) {
        return m_body->ProjectVectorInBody(GetHydrostaticTorqueInWorld(fc),fc);
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