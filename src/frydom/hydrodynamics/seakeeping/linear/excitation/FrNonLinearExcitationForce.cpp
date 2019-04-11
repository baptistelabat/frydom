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

        // Equilibrium frame of the body.
        m_equilibriumFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

        // Wave field.
        auto waveField = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

        // BEMBody.
        auto BEMBody = m_HDB->GetBody(m_body);

        // Frequency and wave direction discretization.
        auto freqs = waveField->GetWaveFrequencies(RADS);
        auto directions = waveField->GetWaveDirections(RAD, NWU, GOTO);

        // Interpolation of the diffraction loads if not already done.
        if (m_Fdiff.empty()) {
            BEMBody->BuildDiffractionInterpolators();
            m_Fdiff = BEMBody->GetExcitationInterp(freqs, directions, RAD); // Yes! Excitation even if only the diffraction loads are returned.
        }

        // Initialization of the parent class.
        FrForce::Initialize();

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

        // Wave field structure.
        auto waveField = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

        // Wave elevation.
        auto complexElevations = waveField->GetComplexElevation(m_equilibriumFrame->GetX(NWU),
                                                              m_equilibriumFrame->GetY(NWU),
                                                              NWU);

        // DOF.
        auto nbMode = m_HDB->GetBody(m_body)->GetNbForceMode();

        // Number of wave frequencies.
        auto nbFreq = waveField->GetWaveFrequencies(RADS).size();

        // Number of wave directions.
        auto nbWaveDir = waveField->GetWaveDirections(RAD, NWU, GOTO).size();

        // Fdiff(t) = eta*Fdiff(Nemoh).
        Eigen::VectorXd forceMode(nbMode);
        forceMode.setZero(); // Initialization.
        for (unsigned int imode=0; imode<nbMode; ++imode) {
            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
                    forceMode(imode) += std::imag(complexElevations[idir][ifreq] * m_Fdiff[idir](imode, ifreq));
                }
            }
        }

        // From vector to force and torque structures.
        auto force = Force();
        auto torque = Torque();

        for (unsigned int imode=0; imode<nbMode; ++imode) {

            auto mode = m_HDB->GetBody(m_body)->GetForceMode(imode);
            Direction direction = mode->GetDirection(); // Unit vector for the force direction.
            switch (mode->GetType()) {
                case FrBEMMode::LINEAR:
                    force += direction * forceMode(imode);
                    break;
                case FrBEMMode::ANGULAR:
                    torque += direction * forceMode(imode);
                    break;
            }
        }
        auto worldForce = m_equilibriumFrame->ProjectVectorFrameInParent(force, NWU);
        auto worldTorque = m_equilibriumFrame->ProjectVectorFrameInParent(torque, NWU);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                        Excitation loads
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Sum of the diffraction and the Froude-Krylov loads.
        worldForce = worldForce + m_FKforce;
        worldTorque = worldTorque + m_FKtorque;

        // Setting the nonlinear Froude-Krylov loads in world at the CoG in world.
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
            mesh::FrMeshTraits::Point Centroid = m_clipped_mesh.data(f_iter).Center();
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
        body->AddExternalForce(excitationForce);

        return excitationForce;

    }

}  // end namespace frydom
