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

        std::vector<Eigen::MatrixXcd> m_Fexc;  // Excitation coefficients interpolated from

        std::vector<std::vector<std::complex<double>>> m_steadyForce;

    public:

        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        void Clear() {
            m_Fexc.clear();
        }

        void Initialize() {

            if (m_Fexc.empty()) {
                // We initialize the Fexc coefficients by interpolation on the Hydrodynamic Database
                auto BEMBody = dynamic_cast<FrHydroBody*>(GetBody())->GetBEMBody();
                auto waveField = m_waveProbe->GetWaveField();
                m_Fexc = BEMBody->GetExcitationInterp(waveField->GetWavePulsations(RADS),
                                                      waveField->GetWaveDirections(DEG),
                                                      DEG);
            }

            // Getting the steady complex elevations



            return;

        }

        void UpdateState() override {

            // Get the wave elevation
//            auto cmplxElevation = m_waveProbe->GetCmplxElevation();
//
//            return;

        }



    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
