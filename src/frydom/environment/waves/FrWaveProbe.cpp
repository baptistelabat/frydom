//
// Created by frongere on 30/10/17.
//

#include "FrWaveField.h"
#include "FrWaveProbe.h"


namespace frydom {


    void FrLinearRegularWaveProbe::Initialize() {
        m_steadyElevation = m_waveField->GetSteadyElevation(m_x, m_y)[0];
    }

    double FrLinearRegularWaveProbe::GetElevation() const {
        auto cmplx_elevation = m_waveField->GetCmplxFreeSurfaceElevation(m_steadyElevation);
        return std::imag(cmplx_elevation);
    }

    std::vector<std::complex<double>> FrLinearRegularWaveProbe::GetCmplxElevation() const {
        std::vector<std::complex<double>> cmplxElevation;
        cmplxElevation.push_back(m_waveField->GetCmplxFreeSurfaceElevation(m_steadyElevation));
        return cmplxElevation;
    }


    void FrLinearIrregularWaveProbe::Initialize() {
        m_steadyElevation = m_waveField->GetSteadyElevation(m_x, m_y);
    }

    double FrLinearIrregularWaveProbe::GetElevation() const {
        auto cmplx_elevation = GetCmplxElevation();
        double elev = 0.;
        for (auto& val: cmplx_elevation) {
            elev += std::imag(val);
        }
        return elev;
    }

    std::vector<std::complex<double>> FrLinearIrregularWaveProbe::GetCmplxElevation() const {
        auto steady = m_steadyElevation;
        return m_waveField->GetCmplxFreeSurfaceElevation(steady);
    }

}  // end namespace frydom