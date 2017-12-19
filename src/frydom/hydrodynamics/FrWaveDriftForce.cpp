//
// Created by camille on 14/12/17.
//

#include "FrWaveDriftForce.h"


namespace frydom {

    FrWaveDriftForce::FrWaveDriftForce(const std::string yaml_file) {

    }

    void FrWaveDriftForce::SetCmplxElevation() {

        auto x = m_waveProbe->GetX();
        auto y = m_waveProbe->GetY();
        m_CmplxElevation = m_waveProbe->GetWaveField()->GetCmplxElevation(x, y);

    }

    void FrWaveDriftForce::UpdateState() {

        std::vector<double> drift_coeff(3);

        auto heading = m_body->GetHeadingAngle(NED);

        auto emjwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();

        std::vector<std::complex<double>> cforce = {0.,0.,0.};

        for (unsigned int ifreq=0; ifreq<emjwt.size(); ++ifreq) {

            drift_coeff = { m_table.Eval("Surge", wi, heading),
                            m_table.Eval("Sway", wi, heading),
                            m_table.Eval("Yaw", wi, heading) };

            cforce += m_CmplxElevation[ifreq] .* drift_coeff * emjwt[ifreq];
        }
        force.at(0) = std::real(cforce.at(0));   //  Surge
        force.at(1) = std::real(cforce.at(1));   //  Sway
        moment.at(3) = std::real(cforce.at(2));  //  Yaw
     }




}