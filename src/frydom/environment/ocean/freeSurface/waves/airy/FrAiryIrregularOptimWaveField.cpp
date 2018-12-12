//
// Created by Lucas Letournel on 11/12/18.
//

#include "FrAiryIrregularOptimWaveField.h"

namespace frydom {

    FrAiryIrregularOptimWaveField::FrAiryIrregularOptimWaveField(frydom::FrFreeSurface_ *freeSurface)
            : FrAiryIrregularWaveField(freeSurface) {

    }

    std::vector<std::vector<Complex>>
    FrAiryIrregularOptimWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if (IsNED(fc)) {y=-y; NWUsign = -NWUsign;}

        std::vector<std::vector<Complex>> ComplexElevation;
        ComplexElevation.reserve(m_nbDir);
        ComplexElevation.clear();

        std::vector<Complex> ComplexElevation_temp;
        ComplexElevation_temp.reserve(m_nbFreq);
        ComplexElevation_temp.clear();

        double ki;
        Complex elevation;

        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            double kdir = x*c_cosTheta[idir] + y*c_sinTheta[idir];
            ComplexElevation_temp.clear();
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                ki = m_waveNumbers[ifreq];
                elevation = exp(JJ * ki * kdir ) * c_expJwt[ifreq] * c_AExpJphi[idir][ifreq] * NWUsign;
                ComplexElevation_temp.push_back(elevation);
            }
            ComplexElevation.push_back(ComplexElevation_temp);
        }

        return ComplexElevation;
    }

    std::vector<mathutils::Vector3d<Complex>>
    FrAiryIrregularOptimWaveField::GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        double NWUsign = 1;
        if (IsNED(fc)) {y=-y; z=-z; NWUsign = -NWUsign;}
        std::vector<mathutils::Vector3d<Complex>> ComplexVel;
        ComplexVel.reserve(m_nbFreq);
        ComplexVel.clear();

        Complex Vx = 0, Vy = 0, Vz = 0;
        double ki, wi;
        double Stretching, StretchingDZ;

        auto ComplexElevation = GetComplexElevation(x,y,fc);

        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            ki = m_waveNumbers[ifreq];
            wi = m_waveFrequencies[ifreq];
            Vx = 0, Vy = 0, Vz = 0;
            Stretching = m_verticalFactor->Eval(x,y,z,ki,c_depth);
            StretchingDZ = m_verticalFactor->EvalDZ(x,y,z,ki,c_depth);
            for (unsigned int idir=0; idir<m_nbDir; ++idir) {
                Vx += c_cosTheta[idir] * wi * ComplexElevation[idir][ifreq] * Stretching;
                Vy += c_sinTheta[idir] * wi * ComplexElevation[idir][ifreq] * Stretching * NWUsign;
                Vz +=   - JJ / ki * wi * ComplexElevation[idir][ifreq] * StretchingDZ * NWUsign;
            }
            ComplexVel.emplace_back(Vx,Vy,Vz);
        }

        return ComplexVel;
    }

    void FrAiryIrregularOptimWaveField::Initialize() {
        FrAiryIrregularWaveField::Initialize();

        c_cosTheta.reserve(m_nbDir); c_cosTheta.clear();
        c_sinTheta.reserve(m_nbDir); c_sinTheta.clear();
        c_AExpJphi.reserve(m_nbDir); c_AExpJphi.clear();

        Complex expTemp;
        double dirTemp;

        std::vector<Complex> Aphi_temp;
        Aphi_temp.reserve(m_nbFreq);
        Aphi_temp.clear();

        auto Amplitudes = m_waveSpectrum->GetWaveAmplitudes(m_waveFrequencies, m_waveDirections);

        for (unsigned int idir=0; idir<m_nbDir; ++idir) {
            dirTemp = cos(m_waveDirections[idir]);
            c_cosTheta.push_back(dirTemp);
            dirTemp = sin(m_waveDirections[idir]);
            c_sinTheta.push_back(dirTemp);
            Aphi_temp.clear();
            for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
                expTemp = Amplitudes[idir][ifreq] * exp(-JJ*m_wavePhases->at(idir)[ifreq]);
                Aphi_temp.push_back(expTemp);
            }
            c_AExpJphi.push_back(Aphi_temp);
        }

        InternalUpdate();
    }

    void FrAiryIrregularOptimWaveField::StepFinalize() {
        FrWaveField_::StepFinalize();
        InternalUpdate();
    }

    void FrAiryIrregularOptimWaveField::InternalUpdate() {
        c_expJwt.empty();
        c_expJwt.reserve(m_nbFreq);
        c_expJwt.clear();

        Complex expTemp;

        for (unsigned int ifreq=0; ifreq<m_nbFreq; ++ifreq) {
            expTemp = exp(-JJ * m_waveFrequencies[ifreq] * c_time);
            c_expJwt.push_back(expTemp);
        }
    }
}