//
// Created by camille on 18/04/18.
//

#include "FrFlowSensor.h"
#include "chrono/core/ChVector.h"

namespace frydom {


    FrFlowSensor::FrFlowSensor(const chrono::ChVector<> pos) {
        m_x = pos.x();
        m_y = pos.y();
        m_z = pos.y();
    }

    void FrFlowSensor::SetPos(const chrono::ChVector<> pos) {
        m_x = pos.x();
        m_y = pos.y();
        m_z = pos.z();
    }

    chrono::ChVector<>* FrFlowSensor::GetPos() const {
        return new chrono::ChVector<double>(m_x, m_y, m_z);
    }

}
