//
// Created by frongere on 31/10/17.
//

#include "FrWaveField.h"
#include "FrWaveProbe.h"
#include "FrFlowSensor.h"

namespace frydom {


    std::shared_ptr<FrLinearWaveProbe> FrLinearWaveField::NewWaveProbe(double x, double y) {
        auto waveProbe = std::make_shared<FrLinearWaveProbe>(x, y);
        waveProbe->SetWaveField(this);
        m_waveProbes.push_back(waveProbe);
        return waveProbe;
    }

    std::shared_ptr<FrLinearFlowSensor> FrLinearWaveField::NewFlowSensor(double x, double y, double z) {
        auto flowSensor = std::make_shared<FrLinearFlowSensor>(this, x, y, z);
        m_flowSensor.push_back(flowSensor);
        return flowSensor;
    }

    std::vector<std::vector<double>> FrLinearWaveField::_GetWaveAmplitudes() const {
        std::vector<std::vector<double>> waveAmplitudes;
        std::vector<double> ampl;
        switch (m_linearWaveType) {

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


    void FrLinearWaveField::SetStretching(FrStretchingType type) {

        switch (type) {
            case NO_STRETCHING:
                m_verticalFactor = std::make_shared<FrKinematicStretching>();
                break;
            case VERTICAL:
                m_verticalFactor = std::make_shared<FrKinStretchingVertical>();
                break;
            case EXTRAPOLATE:
                m_verticalFactor = std::make_shared<FrKinStretchingExtrapol>();
                break;
            case WHEELER:
                m_verticalFactor = std::make_shared<FrKinStretchingWheeler>(this);
                break;
            case CHAKRABARTI:
                m_verticalFactor = std::make_shared<FrKinStretchingChakrabarti>(this);
            case DELTA:
                m_verticalFactor = std::make_shared<FrKinStretchingDelta>(this);
            default:
                m_verticalFactor = std::make_shared<FrKinematicStretching>();
                break;
        }
        m_verticalFactor->SetInfDepth(m_infinite_depth);
    }


    FrFlowSensor* FrWaveField::SetFlowSensor(double x, double y, double z) const {
        return new FrFlowSensor(x, y, z);
    }

    FrFlowSensor* FrWaveField::SetFlowSensor(chrono::ChVector<> pos) const {
        return new FrFlowSensor(pos);
    }


}  // end namespace frydom