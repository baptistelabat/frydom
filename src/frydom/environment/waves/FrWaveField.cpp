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


}  // end namespace frydom