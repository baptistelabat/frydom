//
// Created by frongere on 30/10/17.
//

#include "FrWaveField.h"
#include "FrWaveProbe.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/core/FrNode.h"


namespace frydom {



    // -------------------------------------------------------
    // Linear wave probe
    // -------------------------------------------------------
    std::vector<std::vector<std::complex<double>>> FrLinearWaveProbe::GetCmplxElevation() const {

        bool steady = true;

        auto waveField = dynamic_cast<FrLinearWaveField*>(m_waveField);

        auto nbDir = waveField->GetNbWaveDirections();
        auto nbFreq = waveField->GetNbFrequencies();

        auto waveNumber = waveField->GetWaveNumbers();
        auto waveDir = waveField->GetWaveDirections(RAD);
        auto time = waveField->GetTime();

        // Relative angle and speed of the frame
        auto euler_angles = quat_to_euler(m_node->GetRot(), CARDAN, RAD);
        auto heading = Normalize_0_2PI(euler_angles.z());
        auto speed = m_node->GetPos_dt().Length();

        std::vector<double> velocity;
        velocity.reserve(nbDir);

        // Component of the frame velocity in the wave direction
        for (unsigned int idir=0; idir<nbDir; ++idir) {
            velocity.push_back( speed * cos(waveDir[idir]-heading));
        }

        // Complex elevation in time domain
        auto cmplxElevation = waveField->GetCmplxElevation(GetX(), GetY(), steady);
        auto ejwt = waveField->GetTimeCoeffs();
        for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
            for (unsigned int idir=0; idir<nbDir; ++idir) {
                cmplxElevation[idir][ifreq] *= exp( -JJ * waveNumber[ifreq] * velocity[idir] * time);
            }
        }

        return cmplxElevation;
    }

    double FrLinearWaveProbe::GetElevation(double time) const {

        std::complex<double> elev = 0.;

        auto waveField = dynamic_cast<FrLinearWaveField*>(m_waveField);
        auto cmplxElevation = GetCmplxElevation();

        auto nbDir = waveField->GetNbWaveDirections();
        auto nbFreq = waveField->GetNbFrequencies();

        for (unsigned int idir=0; idir<nbDir; ++idir) {
            for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
                elev += cmplxElevation[idir][ifreq];
            }
        }
        double realElev = imag(elev);

        // Applying the wave ramp
        auto waveRamp = m_waveField->GetWaveRamp();
        if (waveRamp && waveRamp->IsActive()) {
            m_waveField->GetWaveRamp()->Apply(
                    m_waveField->GetTime(),
                    realElev
            );
        }
        return realElev;

    }

    // -------------------------------------------------------
    // Linear wave probe with steady state
    // -------------------------------------------------------

    void FrLinearWaveProbeSteady::Initialize() {
        m_steadyElevation = dynamic_cast<FrLinearWaveField *>(m_waveField)->GetSteadyElevation(m_x, m_y);
    }

    double FrLinearWaveProbeSteady::GetElevation(double time) const {

        auto emjwt = dynamic_cast<FrLinearWaveField *>(m_waveField)->GetTimeCoeffs(); // FIXME: tres couteux a l'appel...
        std::complex<double> elev = 0.;
        for (unsigned int ifreq = 0; ifreq < emjwt.size(); ++ifreq) {
            elev += m_steadyElevation[ifreq] * emjwt[ifreq];
        }
        double realElev = imag(elev);

        // Applying the wave ramp
        auto waveRamp = m_waveField->GetWaveRamp();
        if (waveRamp && waveRamp->IsActive()) {
            m_waveField->GetWaveRamp()->Apply(
                    m_waveField->GetTime(),
                    realElev
            );
        }
        return realElev;
    }


}  // end namespace frydom