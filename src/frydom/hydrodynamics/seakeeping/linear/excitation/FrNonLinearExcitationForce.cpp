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


#include "FrNonLinearExcitationForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    void FrNonLinearExcitationForce::Initialize() {

        // Initialization of the parent class.
        FrExcitationForceBase::Initialize();

    }

    void FrNonLinearExcitationForce::Compute(double time) {

        // This subroutine computes the nonlinear excitation forces (nonlinear RK, linear diffraction) from Nemoh results and pressure integration.

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                      Froude-Krylov loads
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Clipped mesh.
        m_clipped_mesh = m_hydro_mesh->GetClippedMesh();

        // Computation of the Froude-Krylov force.
        CalcIncidentPressureIntegration();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                        Diffraction loads
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Computation of the diffraction loads.
        Compute_F_HDB();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                        Excitation loads
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Sum of the diffraction and the Froude-Krylov loads.
        Force worldForce = m_WorldForce + m_FKforce;
        Torque worldTorque = m_WorldTorque + m_FKtorque;

        // Setting the nonlinear excitation loads in world at the CoG in world.
        this->SetForceTorqueInWorldAtCOG(worldForce, worldTorque, NWU);

	    // Settings: torque is already computed at CoG.
        SetForceTorqueInWorldAtCOG(worldForce,worldTorque, NWU);

    }

    void FrNonLinearExcitationForce::CalcIncidentPressureIntegration(){

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
        for (mesh::FrMesh::FaceIter f_iter = m_clipped_mesh.faces_begin(); f_iter != m_clipped_mesh.faces_end(); ++f_iter) {

            // Normal.
            Normal = m_clipped_mesh.normal(*f_iter);
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
            Pressure = m_free_surface->GetPressure(Centroid[0],Centroid[1],Centroid[2],NWU);

            // Area.
            Area = m_clipped_mesh.GetArea(*f_iter);

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

    Eigen::MatrixXcd FrNonLinearExcitationForce::GetHDBData(unsigned int iangle) const {

        auto BEMBody = m_HDB->GetBody(m_body);

        return BEMBody->GetDiffraction(iangle);

    }

    Eigen::VectorXcd FrNonLinearExcitationForce::GetHDBData(unsigned int iangle, unsigned int iforce) const {

        auto BEMBody = m_HDB->GetBody(m_body);

        return BEMBody->GetDiffraction(iangle,iforce);

    }

    void FrNonLinearExcitationForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writing the clipped mesh in an output file.
//        m_clipped_mesh.Write("Mesh_clipped_Froude_Krylov.obj");
    }

    std::shared_ptr<FrNonLinearExcitationForce>
    make_nonlinear_excitation_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::shared_ptr<FrHydroMesh> HydroMesh){

        // This subroutine creates a (fully or weakly) nonlinear excitation force object.

        // Construction of the (fully or weakly) excitation force object from the HDB.
        auto excitationForce = std::make_shared<FrNonLinearExcitationForce>(body->GetSystem(),HDB,HydroMesh);

        // Add the excitation force object as an external force to the body.
        body->AddExternalForce(excitationForce); // Initialization of m_body.

        return excitationForce;

    }

}  // end namespace frydom
