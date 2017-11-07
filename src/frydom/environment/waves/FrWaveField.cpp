//
// Created by frongere on 31/10/17.
//

#include "FrWaveField.h"
#include "FrWaveProbe.h"

namespace frydom {


    std::shared_ptr<FrLinearWaveProbe> FrLinearWaveField::NewWaveProbe(double x, double y) {
        auto waveProbe = std::make_shared<FrLinearWaveProbe>(x, y);
        waveProbe->SetWaveField(this);
        m_waveProbes.push_back(waveProbe);
        return waveProbe;
    }

    std::vector<std::vector<double>> FrLinearWaveField::_GetWaveAmplitudes() const {
        std::vector<std::vector<double>> waveAmplitudes;
        std::vector<double> ampl;
        switch (m_type) {

            case LINEAR_REGULAR:
                ampl.push_back(m_height * 0.5);
                waveAmplitudes.push_back(ampl);
                break;

            case LINEAR_IRREGULAR:
                ampl = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq);
                waveAmplitudes.push_back(ampl);
                break;

            case LINEAR_DIRECTIONAL:
                waveAmplitudes = m_waveSpectrum->GetWaveAmplitudes(m_nbFreq, m_minFreq, m_maxFreq,
                                                                   m_nbDir, m_minDir, m_maxDir, m_meanDir);
                break;
        }
        return waveAmplitudes;
    }


}  // end namespace frydom