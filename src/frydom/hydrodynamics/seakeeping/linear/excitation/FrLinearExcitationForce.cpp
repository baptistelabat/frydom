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

/// <<<<<<<<<<<<<< REFACTORING INCLUDE

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"
#include "frydom/core/math/functions/ramp/FrLinearRampFunction.h"


namespace frydom {

//    void FrLinearExcitationForce::Initialize() {
//
//        auto BEMBody = GetBEMBody();
//
//        auto waveField = m_waveProbe->GetWaveField();
//
//        if (m_Fexc.empty()) {
//            // We initialize the Fexc coefficients by interpolation on the Hydrodynamic Database
//            m_Fexc = BEMBody->GetExcitationInterp(waveField->GetWaveFrequencies(RADS),
//                                                  waveField->GetWaveDirections(DEG),
//                                                  DEG);
//        }
//
//        // Getting the steady complex elevations
//        bool steady = true;
//        double x = m_waveProbe->GetX();
//        double y = m_waveProbe->GetY();
//        auto cmplxElevations = waveField->GetCmplxElevation(x, y, steady);
//
//        // Computing the steady force
//        auto nbFreq = waveField->GetNbFrequencies();
//        auto nbWaveDir = waveField->GetNbWaveDirections();
//        auto nbForceModes = BEMBody->GetNbForceMode();
//
//        m_steadyForce.resize(nbForceModes, nbFreq);
//        m_steadyForce.setZero();
//
//        for (unsigned int imode=0; imode<nbForceModes; ++imode) {
//            for (unsigned int  ifreq=0; ifreq<nbFreq; ++ifreq) {
//                for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
//                    m_steadyForce(imode, ifreq) += cmplxElevations[idir][ifreq] * m_Fexc[idir](imode, ifreq);
//                }
//            }
//        }
//
//        FrForce::Initialize();
//    }
//
//
//    void FrLinearExcitationForce::SetSteadyForce() {
//
//        auto BEMBody = GetBEMBody();
//
//        auto waveField = m_waveProbe->GetWaveField();
//
//        // Getting the steady complex elevations
//        bool steady = true;
//        double x = m_waveProbe->GetX();
//        double y = m_waveProbe->GetY();
//        auto cmplxElevations = waveField->GetCmplxElevation(x, y, steady);
//
//        // Computing the steady force
//        auto nbFreq = waveField->GetNbFrequencies();
//        auto nbWaveDir = waveField->GetNbWaveDirections();
//        auto nbForceModes = BEMBody->GetNbForceMode();
//
//        m_steadyForce.resize(nbForceModes, nbFreq);
//        m_steadyForce.setZero();
//
//        for (unsigned int imode=0; imode<nbForceModes; ++imode) {
//            for (unsigned int  ifreq=0; ifreq<nbFreq; ++ifreq) {
//                for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
//                    m_steadyForce(imode, ifreq) += cmplxElevations[idir][ifreq] * m_Fexc[idir](imode, ifreq);
//                }
//            }
//        }
//    }
//
//    void FrLinearExcitationForce::UpdateState() {
//
//        auto BEMBody = GetBEMBody();
//
//        auto ejwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();
//
//        if (update_position) {                                      // FIXME : a voir si laisser comme ça ou passer dans une autre classe
//            auto mybody = dynamic_cast<FrHydroBody*>(GetBody());
//            auto eqFrame = mybody->GetEquilibriumFrame();
//            m_waveProbe->SetX(eqFrame->GetPos().x());
//            m_waveProbe->SetY(eqFrame->GetPos().y());
//            auto waveField = m_waveProbe->GetWaveField();
//
//            SetSteadyForce();
//        }
//
//
//        auto nbMode = BEMBody->GetNbForceMode();
////            auto nbFreq = BEMBody->GetNbFrequencies();
//
//
//        auto nbFreq = ejwt.size();
//
//        Eigen::VectorXd forceMode(nbMode);
//        forceMode.setZero();
//
//        for (unsigned int imode=0; imode<nbMode; ++imode) {
//            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
//                forceMode(imode) += std::imag(m_steadyForce(imode, ifreq) * ejwt[ifreq]);
//            }
//        }
//
//        // On teste pour le moment du 6ddl (nbModes == 6)
//
//
//        // Applying the wave Ramp
//        auto waveRamp = m_waveProbe->GetWaveField()->GetWaveRamp();
//        if (waveRamp && waveRamp->IsActive()) {
//            for (unsigned int imode=0; imode<nbMode; ++imode) {
//                forceMode[imode] *= waveRamp->Get_y(ChTime); // TODO: WaveRamp doit pouvoir s'appliquer a des vecteurs...
////                waveRamp->Apply(ChTime, forceMode[imode]); // TODO: WaveRamp doit pouvoir s'appliquer a des vecteurs...
//            }
//        }
//
//        force.SetNull();
//        moment.SetNull();
//
//        FrBEMMode* mode;
//        chrono::ChVector<double> direction;
//        chrono::ChVector<double> point;
//
//        FrBEMMode::TYPE modeType;
//
//        // FIXME: les deux boucles suivantes ne sont vraiment pas optimales !!!!
//
//        // Linear force computation
//        for (unsigned int imode=0; imode<nbMode; ++imode) {
//            mode = BEMBody->GetForceMode(imode);
//            modeType = mode->GetType();
//
//            direction = ChEig(mode->GetDirection());
//            if (modeType == FrBEMMode::LINEAR) {
//                force += forceMode(imode) * direction;
//            }
//        }
//
//        // Moment computation
//        for (unsigned int imode=0; imode<nbMode; ++imode) {
//            mode = BEMBody->GetForceMode(imode);
//            modeType = mode->GetType();
//
//            direction = ChEig(mode->GetDirection());
//            point = ChEig(mode->GetPoint());
//            if (modeType == FrBEMMode::ANGULAR) {
////                    moment += forceMode(imode) * direction + point.Cross(force);
//                moment += forceMode(imode) * direction;
//            }
//        }  // FIXME: verifier qu'on ne fait pas d'erreur dans le calcul du moment...
//
//        // TODO: voir comment faire pour restreindre des ddls...
//
//        moment = GetBody()->Dir_World2Body(moment);
//
////            moment.SetNullRotation(); // Retirer !!!
//
//    }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

    void FrLinearExcitationForce_::Initialize() {


        /// This subroutine initializes the excitation force object.

        // Initialization of the parent class.
//        FrForce_::Initialize();

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
        FrForce_::Initialize();

    }

    void FrLinearExcitationForce_::Update(double time) {

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
                case FrBEMMode_::LINEAR:
                    force += direction * forceMode(imode);
                    break;
                case FrBEMMode_::ANGULAR:
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

    void FrLinearExcitationForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }


    std::shared_ptr<FrLinearExcitationForce_>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB_> HDB, std::shared_ptr<FrBody_> body){

        /// This subroutine creates the hydrostatic force object for computing the hydrostatic loads.

        // Construction of the excitation force object from the HDB.
        auto excitationForce = std::make_shared<FrLinearExcitationForce_>(HDB);

        // Add the excitation force object as an external force to the body.
        body->AddExternalForce(excitationForce);

        return excitationForce;

    }

}  // end namespace frydom
