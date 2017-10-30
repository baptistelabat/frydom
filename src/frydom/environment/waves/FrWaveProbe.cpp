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





    void FrLinearIrregularWaveProbe::Initialize() {
        m_steadyElevation = m_waveField->GetSteadyElevation(m_x, m_y);
    }

    double FrLinearIrregularWaveProbe::GetElevation() const {
        auto steady = m_steadyElevation;  // TODO: pourquoi on ne peut pas directement place steady dans la methode ?
        auto cmplx_elevation = m_waveField->GetCmplxFreeSurfaceElevation(steady);
        double elev = 0.;
        for (auto& val: cmplx_elevation) {
            elev += std::imag(val);
        }
        return elev;
    }
}  // end namespace frydom