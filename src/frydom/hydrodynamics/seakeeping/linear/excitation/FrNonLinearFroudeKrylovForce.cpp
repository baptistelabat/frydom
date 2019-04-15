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

        // Clipped mesh.
        m_clipped_mesh = m_hydro_mesh->GetClippedMesh();

        // Computation of the Froude-Krylov force.
        CalcIncidentPressureIntegration();

        // Setting the nonlinear excitation loads in world at the CoG in world.
        this->SetForceTorqueInWorldAtCOG(m_FKforce, m_FKtorque, NWU);

	    // Settings: torque is already computed at CoG.
        SetForceTorqueInWorldAtCOG(m_FKforce,m_FKtorque, NWU);

    }

    void FrNonLinearFroudeKrylovForce::CalcIncidentPressureIntegration(){

        // This function performs the incident pressure integration.

        mesh::FrMesh::Normal Normal;
        double Pressure,Area,PA;
        m_FKforce = Force(0.,0.,0.);
        m_FKtorque = Torque(0.,0.,0.);
        Position CoG = m_body->GetCOG(NWU);
        Position CentroidPos,NormalPos;
        Position GM;
        Position GMvectNormal;

        // Loop over the faces.
        for (auto& f_iter : m_clipped_mesh.faces()) {

            // Normal.
            Normal = m_clipped_mesh.normal(f_iter);
            NormalPos[0] = Normal[0];
            NormalPos[1] = Normal[1];
            NormalPos[2] = Normal[2];

            // Centroid (where the pressure is evaluated).
            mesh::FrMeshTraits::Point Centroid = m_clipped_mesh.data(f_iter).Center(); // Here a warning at the compilation step.
            CentroidPos[0] = Centroid[0];
            CentroidPos[1] = Centroid[1];
            CentroidPos[2] = Centroid[2];

            // Incident pressure.
            // The pressure is assumed constant over a panel.
            Pressure = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetPressure(Centroid[0],Centroid[1],Centroid[2],NWU);

            // Area.
            Area = m_clipped_mesh.GetArea(f_iter);

            // Pressure * Area.
            PA = -Pressure*Area;

            // Froude-Krylov force.
            m_FKforce[0] = m_FKforce[0] + PA*Normal[0];
            m_FKforce[1] = m_FKforce[1] + PA*Normal[1];
            m_FKforce[2] = m_FKforce[2] + PA*Normal[2];

            // GM vect n;
            GM = CentroidPos - CoG;
            GMvectNormal = GM.cross(NormalPos);

            // Froude-Krylov torque.
            m_FKtorque[0] = m_FKtorque[0] + PA*GMvectNormal[0];
            m_FKtorque[1] = m_FKtorque[1] + PA*GMvectNormal[1];
            m_FKtorque[2] = m_FKtorque[2] + PA*GMvectNormal[2];

        }

    }

    void FrNonLinearFroudeKrylovForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writing the clipped mesh in an output file.
//        m_clipped_mesh.Write("Mesh_clipped_Froude_Krylov.obj");
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
