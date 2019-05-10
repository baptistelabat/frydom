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

        SetForceInWorldAtPointInWorld(GetHydrostaticForceInWorld(NWU), GetCenterOfBuoyancyInWorld(NWU), NWU);

    }

    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){
        return m_body->GetPointPositionInBody(GetCenterOfBuoyancyInWorld(fc),fc);
    }

    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc) {

        // clipped mesh is expressed in the world reference frame, but its horizontal position is centered around (0.,0.)
        auto CoBInWorld = m_hydroMesh->GetClippedMesh().GetCOG();

        // Addition of the horizontal position of the body
        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;
        CoBInWorld += bodyPos;

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);

        return CoBInWorld;
    }

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