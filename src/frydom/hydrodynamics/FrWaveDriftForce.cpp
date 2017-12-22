//
// Created by camille on 14/12/17.
//

#include "frydom/environment/waves/FrWaveField.h"
#include "FrWaveDriftForce.h"


namespace frydom {

    FrWaveDriftForce::FrWaveDriftForce(const std::string hdf5_file) {

    }

    void FrWaveDriftForce::SetCmplxElevation() {

        auto x = m_waveProbe->GetX();
        auto y = m_waveProbe->GetY();

        m_CmplxElevation = m_waveProbe->GetWaveField()->GetCmplxElevation(x, y, true);

    }

    void FrWaveDriftForce::UpdateState() {

        //std::vector<double> drift_coeff(3); //FIXME : dynamic behaviour

        auto nbFreq = m_waveProbe->GetWaveField()->GetNbFrequencies();
        auto nbWaveDir = m_waveProbe->GetWaveField()->GetNbWaveDirections();

        auto heading = m_body->GetHeadingAngle(NED);

        auto emjwt = m_waveProbe->GetWaveField()->GetTimeCoeffs();

        std::vector<std::complex<double>> cforce = {0.,0.,0.};

        auto waveDir = m_waveProbe->GetWaveField()->GetWaveDirections(RAD);
        auto w = m_waveProbe->GetWaveField()->GetWaveFrequencies(RADS);
        double wi, relative_angle;

        for (unsigned int idir=0; idir<nbWaveDir; ++idir) {
            for (unsigned int ifreq=0; ifreq < nbFreq; ++ifreq) {

                wi  =w[ifreq];
                relative_angle = heading - waveDir[idir];

                //drift_coeff = {m_table.Eval("Surge", wi, relative_angle),
                //               m_table.Eval("Sway", wi, relative_angle),
               //               m_table.Eval("Yaw", wi, relative_angle)};

                //cforce += m_CmplxElevation[idir][ifreq] .* drift_coeff * emjwt[ifreq];

                cforce.at(0) += m_CmplxElevation[idir][ifreq] * m_table.Eval("Surge", wi, relative_angle) * emjwt[ifreq];
                cforce.at(1) += m_CmplxElevation[idir][ifreq] * m_table.Eval("Sway", wi, relative_angle) * emjwt[ifreq];
                cforce.at(2) += m_CmplxElevation[idir][ifreq] * m_table.Eval("Yaw", wi, relative_angle) * emjwt[ifreq];

            }
        }
        force.x() = std::real(cforce.at(0));   //  Surge
        force.y() = std::real(cforce.at(1));   //  Sway
        moment.z() = std::real(cforce.at(2));  //  Yaw
     }




}