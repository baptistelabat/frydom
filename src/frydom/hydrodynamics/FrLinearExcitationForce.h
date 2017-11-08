//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/waves/FrWaveProbe.h>
#include "frydom/environment/waves/FrWaveField.h"
#include "frydom/core/FrHydroBody.h"
#include "FrHydroDB.h"

//#include "Eigen/Dense"

namespace frydom {

    class FrLinearExcitationForce : public FrForce {

    private:

        std::shared_ptr<FrLinearWaveProbe> m_waveProbe;

        std::vector<Eigen::MatrixXcd> m_Fexc;  // Excitation coefficients interpolated from the database

        Eigen::MatrixXcd m_steadyForce;

    public:

        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        void Clear() {
            m_Fexc.clear();
        }

        void Initialize() {

            auto BEMBody = dynamic_cast<FrHydroBody*>(GetBody())->GetBEMBody();
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

        }

        void UpdateState() override {
            auto BEMBody = dynamic_cast<FrHydroBody*>(GetBody())->GetBEMBody();

            auto nbMode = BEMBody->GetNbForceMode();
            auto nbFreq = BEMBody->GetNbFrequencies();

            auto ejwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();

            Eigen::VectorXd forceMode(nbMode);
            forceMode.setZero();

            for (unsigned int imode=0; imode<nbMode; ++imode) {
                for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                    forceMode(imode) += std::imag(m_steadyForce(imode, ifreq) * ejwt[ifreq]);
                }
            }

            force.SetNull();
            moment.SetNull();

            FrBEMMode* mode;
            chrono::ChVector<double> direction;
            chrono::ChVector<double> point;

            FrBEMMode::TYPE modeType;

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
                    moment += forceMode(imode) * direction + point.Cross(force);
                }
            }  // FIXME: verifier qu'on ne fait pas d'erreur dans le calcul du moment...

            // TODO: voir comment faire pour restreindre des ddls...

            moment = GetBody()->Dir_World2Body(moment);

        }



    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
