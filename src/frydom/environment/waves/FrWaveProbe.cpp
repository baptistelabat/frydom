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

        bool steady = false;

        auto waveField = dynamic_cast<FrLinearWaveField*>(m_waveField);

        auto nbDir = waveField->GetNbWaveDirections();
        auto nbFreq = waveField->GetNbFrequencies();

        auto waveNumber = waveField->GetWaveNumbers();
        auto waveDir = waveField->GetWaveDirections(RAD);
        auto time = waveField->GetTime();

        // Relative angle and speed of the frame
        auto frame_velocity = m_node->GetPos_dt();
        auto angle = Normalize_0_2PI(atan2(frame_velocity.y(), frame_velocity.x()));
        auto norm_speed = frame_velocity.Length();

        std::vector<double> velocity;
        velocity.reserve(nbDir);

        // Component of the frame velocity in the wave direction
        for (unsigned int idir=0; idir<nbDir; ++idir) {
            velocity.push_back( norm_speed * cos(waveDir[idir]-angle));
        }

        // Complex elevation in time domain
        auto cmplxElevation = waveField->GetCmplxElevation(GetX(), GetY(), steady);
        /*
        for (unsigned int ifreq=0; ifreq<nbFreq; ++ifreq) {
            for (unsigned int idir=0; idir<nbDir; ++idir) {
                cmplxElevation[idir][ifreq] *= exp( +JJ * waveNumber[ifreq] * velocity[idir] * time);
            }
        }
        */

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

    std::vector<double> FrLinearWaveProbe::GetFrequencies() const {

        auto waveField = dynamic_cast<FrLinearWaveField*>(m_waveField);

        auto nbDir = waveField->GetNbWaveDirections();
        auto nbFreq = waveField->GetNbFrequencies();

        auto waveNumber = waveField->GetWaveNumbers();
        auto waveDir = waveField->GetWaveDirections(RAD);

        // Relative angle and speed of the frame
        auto frame_velocity = m_node->GetPos_dt();
        auto angle = Normalize_0_2PI(atan2(frame_velocity.y(), frame_velocity.x()));
        auto norm_speed = frame_velocity.Length();

        std::vector<double> velocity;
        velocity.reserve(nbDir);

        // Component of the frame velocity in the wave direction
        for (unsigned int idir=0; idir<nbDir; ++idir) {
            velocity.push_back( norm_speed * cos(waveDir[idir]-angle));
        }

    }

    std::vector<std::vector<double>> FrLinearWaveProbe::GetEncounterWaveFrequencies() const {

        std::vector<std::vector<double>> waveEncounterFrequencies;
        std::vector<double> waveEncounterFrequencies_freq;

        auto waveField = dynamic_cast<FrLinearWaveField*>(m_waveField);

        auto nbDir = waveField->GetNbWaveDirections();
        auto nbFreq = waveField->GetNbFrequencies();

        auto waveNumber = waveField->GetWaveNumbers();
        auto waveDir = waveField->GetWaveDirections(RAD);
        auto waveFrequencies = waveField->GetWaveFrequencies(RADS);

        // Relative angle and speed of the frame
        auto frame_velocity = m_node->GetPos_dt();
        auto angle = Normalize_0_2PI(atan2(frame_velocity.y(), frame_velocity.x()));
        auto norm_speed = frame_velocity.Length();

        std::vector<double> velocity;
        velocity.reserve(nbDir);

        // Component of the frame velocity in the wave direction
        for (unsigned int idir=0; idir<nbDir; ++idir) {
            velocity.push_back( norm_speed * cos(waveDir[idir]-angle));
        }

        // Encounter frequencies

        waveEncounterFrequencies.reserve(nbDir);
        for (unsigned int idir=0; idir<nbDir; idir++) {
            waveEncounterFrequencies_freq.clear();
            waveEncounterFrequencies_freq.reserve(nbFreq);
            for (unsigned int ifreq=0; ifreq<nbFreq; ifreq++) {
                waveEncounterFrequencies_freq.push_back(waveFrequencies[ifreq] - waveNumber[ifreq] * velocity[idir]);
            }
            waveEncounterFrequencies.push_back(waveEncounterFrequencies_freq);
        }

        return waveEncounterFrequencies;
    }


    // -------------------------------------------------------
    // Linear wave probe with steady state
    // -------------------------------------------------------

    void FrLinearWaveProbeSteady::Initialize() {
        m_steadyElevation = dynamic_cast<FrLinearWaveField *>(m_waveField)->GetSteadyElevation(GetX(), GetY());
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

    std::vector<double> FrLinearWaveProbeSteady::GetFrequencies() const {
        auto waveField = dynamic_cast<FrLinearWaveField *>(m_waveField);
        return waveField->GetWaveFrequencies();
    }


}  // end namespace frydom