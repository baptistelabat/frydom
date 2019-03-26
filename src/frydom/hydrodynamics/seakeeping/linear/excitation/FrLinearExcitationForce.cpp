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

#include "FrLinearExcitationForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

namespace frydom {

    void FrLinearExcitationForce::Initialize() {

        // Equilibrium frame of the body.
        m_equilibriumFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

        // Wave field.
        auto waveField = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

        // BEMBody.
        auto BEMBody = m_HDB->GetBody(m_body);

        // Frequency and wave direction discretization.
        auto freqs = waveField->GetWaveFrequencies(RADS);
        auto directions = waveField->GetWaveDirections(RAD, NWU, GOTO);

        // Interpolation of the exciting loads if not already done.
        if (m_Fexc.empty()) {
            m_Fexc = BEMBody->GetExcitationInterp(freqs, directions, RAD);
        }

        // Initialization of the parent class.
        FrForce::Initialize();

    }

    void FrLinearExcitationForce::Update(double time) {

        // This subroutine computes the linear excitation forces from Nemoh results.

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

        // Fexc(t) = eta*Fexc(Nemoh).
        Eigen::VectorXd forceMode(nbMode);
        forceMode.setZero(); // Initialization.
        for (unsigned int imode=0; imode<nbMode; ++imode) {
            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
                    forceMode(imode) += std::imag(complexElevations[idir][ifreq] * m_Fexc[idir](imode, ifreq));
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

        this->SetForceTorqueInWorldAtCOG(worldForce, worldTorque, NWU);


	    // Settings: torque is already computed at CoG.
        SetForceTorqueInWorldAtCOG(worldForce,worldTorque, NWU);
    }

    void FrLinearExcitationForce::StepFinalize() {
        FrForce::StepFinalize();
    }

    std::shared_ptr<FrLinearExcitationForce>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body){

        /// This subroutine creates the linear excitation force object.

        // Construction of the excitation force object from the HDB.
        auto excitationForce = std::make_shared<FrLinearExcitationForce>(HDB);

        // Add the excitation force object as an external force to the body.
        body->AddExternalForce(excitationForce);

        return excitationForce;

    }

}  // end namespace frydom
