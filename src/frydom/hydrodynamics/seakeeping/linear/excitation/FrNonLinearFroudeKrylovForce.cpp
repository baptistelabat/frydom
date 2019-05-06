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


#include "FrNonLinearFroudeKrylovForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    void FrNonLinearFroudeKrylovForce::Initialize() {

        // Initialization of the parent class.
        FrForce::Initialize();

    }

    void FrNonLinearFroudeKrylovForce::Compute(double time) {

        // This function computes the fully or weakly nonlinear Froude-Krylov forces from the pressure integration.

        // Computate of the Froude-Krylov force and torque loads at CoG
        CalcIncidentPressureIntegration();

        // Set the force and torque loads at CoG
        SetForceTorqueInWorldAtCOG(m_FKforce,m_FKtorque, NWU);

    }

    void FrNonLinearFroudeKrylovForce::CalcIncidentPressureIntegration(){

        // This function performs the incident pressure integration.
        m_FKforce.setZero();
        m_FKtorque.setZero();
        Position CoG = m_body->GetCOG(NWU);
        Position NormalPos;

        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
        
        auto waveField = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

        // Loop over the faces.
        for (auto& f_iter : clippedMesh->faces()) {

            // Normal
            NormalPos.GetX() = clippedMesh->normal(f_iter)[0];
            NormalPos.GetY() = clippedMesh->normal(f_iter)[1];
            NormalPos.GetZ() = clippedMesh->normal(f_iter)[2];

            // Centroid (where the pressure is evaluated).
            Position Centroid = mesh::OpenMeshPointToVector3d<Position>(clippedMesh->data(f_iter).Center());

            // Incident pressure.
            // The pressure is assumed constant over a panel.
            double Pressure = waveField->GetPressure(Centroid.GetX(),Centroid.GetY(),Centroid.GetZ(),NWU);

            // Area.
            double Area = clippedMesh->GetArea(f_iter);

            // Pressure * Area.
            double PA = -Pressure*Area;

            // Froude-Krylov force.
            m_FKforce += PA*NormalPos;

            // GM vect n;
            auto GM = Centroid - CoG;
            auto GMvectNormal = GM.cross(NormalPos);

            // Froude-Krylov torque.
            m_FKtorque += PA*GMvectNormal;

        }

    }

    void FrNonLinearFroudeKrylovForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writing the clipped mesh in an output file.
//        m_clippedMesh->Write("Mesh_clipped_Froude_Krylov.obj");
    }

    std::shared_ptr<FrNonLinearFroudeKrylovForce>
    make_nonlinear_froude_krylov_force(std::shared_ptr<FrBody> body, std::shared_ptr<FrHydroMesh> HydroMesh){

        // This function creates a fully or weakly nonlinear Froude-Krylov force object.

        // Construction of the fully or weakly Froude-Krylov force object from the HDB.
        auto NonlinFKForce = std::make_shared<FrNonLinearFroudeKrylovForce>(HydroMesh);

        // Add the Froude-Krylov force object as an external force to the body.
        body->AddExternalForce(NonlinFKForce); // Initialization of m_body.

        return NonlinFKForce;

    }

}  // end namespace frydom
