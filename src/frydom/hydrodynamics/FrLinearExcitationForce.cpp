//
// Created by frongere on 30/10/17.
//

#include "FrLinearExcitationForce.h"


namespace frydom {

    void FrLinearExcitationForce::Initialize() {

        auto BEMBody = GetBEMBody();

        auto waveField = m_waveProbe->GetWaveField();

        if (m_Fexc.empty()) {
            // We initialize the Fexc coefficients by interpolation on the Hydrodynamic Database
            m_Fexc = BEMBody->GetExcitationInterp(waveField->GetWaveFrequencies(RADS),
                                                  waveField->GetWaveDirections(DEG),
                                                  DEG);
        }

        // Getting the steady complex elevations
        bool steady = true;
        double x = m_waveProbe->GetX();
        double y = m_waveProbe->GetY();
        auto cmplxElevations = waveField->GetCmplxElevation(x, y, steady);

        // Computing the steady force
        auto nbFreq = waveField->GetNbFrequencies();
        auto nbWaveDir = waveField->GetNbWaveDirections();
        auto nbForceModes = BEMBody->GetNbForceMode();

        m_steadyForce.resize(nbForceModes, nbFreq);
        m_steadyForce.setZero();

        for (unsigned int imode=0; imode<nbForceModes; ++imode) {
            for (unsigned int  ifreq=0; ifreq<nbFreq; ++ifreq) {
                for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
                    m_steadyForce(imode, ifreq) += cmplxElevations[idir][ifreq] * m_Fexc[idir](imode, ifreq);
                }
            }
        }

        // ##CC : debug excitation force
        //std::cout << " ################# STEADY FORCE " << std::endl;
        //std::cout << " elevation : " << cmplxElevations[0][0];
        //std::cout << ", Fe : " << m_Fexc[0](0, 0);
        //std::cout << ", steadyForce : " << m_steadyForce(0, 0);
        //std::cout << ", ejwt : " << waveField->GetTimeCoeffs()[0];
        //std::cout << std::endl;
        // ##CC
    }


    void FrLinearExcitationForce::UpdateState() {

        auto BEMBody = GetBEMBody();

        auto ejwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();

        if (update_position) {                                      // FIXME : a voir si laisser comme ça ou passer dans une autre classe
            auto mybody = dynamic_cast<FrHydroBody*>(GetBody());
            auto eqFrame = mybody->GetEquilibriumFrame();
            m_waveProbe->SetX(eqFrame->GetPos().x());
            m_waveProbe->SetY(eqFrame->GetPos().y());
            auto waveField = m_waveProbe->GetWaveField();
            //m_Fexc = BEMBody->GetExcitationInterp(waveField->GetWaveFrequencies(RADS),
            //                                      waveField->GetWaveDirections(DEG),
            //                                      waveField->GetWaveNumbers(),
            //                                      eqFrame->GetPos_dt(),
            //                                      DEG);
            Initialize();
        }


        auto nbMode = BEMBody->GetNbForceMode();
//            auto nbFreq = BEMBody->GetNbFrequencies();


        auto nbFreq = ejwt.size();

        Eigen::VectorXd forceMode(nbMode);
        forceMode.setZero();

        for (unsigned int imode=0; imode<nbMode; ++imode) {
            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                forceMode(imode) += std::imag(m_steadyForce(imode, ifreq) * ejwt[ifreq]);
            }
        }

        // On teste pour le moment du 6ddl (nbModes == 6)


        // Applying the wave Ramp
        auto waveRamp = m_waveProbe->GetWaveField()->GetWaveRamp();
        if (waveRamp && waveRamp->IsActive()) {
            for (unsigned int imode=0; imode<nbMode; ++imode) {
                waveRamp->Apply(ChTime, forceMode[imode]); // TODO: WaveRamp doit pouvoir s'appliquer a des vecteurs...
            }
        }

        force.SetNull();
        moment.SetNull();

        FrBEMMode* mode;
        chrono::ChVector<double> direction;
        chrono::ChVector<double> point;

        FrBEMMode::TYPE modeType;

        // FIXME: les deux boucles suivantes ne sont vraiment pas optimales !!!!

        // Linear force computation
        for (unsigned int imode=0; imode<nbMode; ++imode) {
            mode = BEMBody->GetForceMode(imode);
            modeType = mode->GetType();

            direction = ChEig(mode->GetDirection());
            if (modeType == FrBEMMode::LINEAR) {
                force += forceMode(imode) * direction;
            }
        }

        // Moment computation
        for (unsigned int imode=0; imode<nbMode; ++imode) {
            mode = BEMBody->GetForceMode(imode);
            modeType = mode->GetType();

            direction = ChEig(mode->GetDirection());
            point = ChEig(mode->GetPoint());
            if (modeType == FrBEMMode::ANGULAR) {
//                    moment += forceMode(imode) * direction + point.Cross(force);
                moment += forceMode(imode) * direction;
            }
        }  // FIXME: verifier qu'on ne fait pas d'erreur dans le calcul du moment...

        // TODO: voir comment faire pour restreindre des ddls...

        moment = GetBody()->Dir_World2Body(moment);

//            moment.SetNull(); // Retirer !!!

    }




}  // end namespace frydom