// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include "FrPositionRecorder.h"


namespace frydom {


    void FrPositionRecorder::SetSize(unsigned int size) { m_size = size; }

    unsigned int FrPositionRecorder::GetSize() const { return m_size; }

    void FrPositionRecorder::SetBody(FrBody *body) { m_body = body; }

    FrBody *FrPositionRecorder::GetBody() const { return m_body; }

    void FrPositionRecorder::Initialize() {
        m_positions.reserve(3);
        for (unsigned int i=0; i<3; i++) {
            m_positions.emplace_back(boost::circular_buffer<double>(m_size, m_size, 0.));
        }
    }

    double FrPositionRecorder::GetTime() const { return m_lastTime; }

    void FrPositionRecorder::RecordPosition() {

        auto currentTime = m_body->GetChTime();
        if (m_lastTime == currentTime) return;
        m_lastTime = currentTime;

        auto linear_position = m_body->GetPos();

        m_positions[0].push_front(linear_position[0]);
        m_positions[1].push_front(linear_position[1]);
        m_positions[2].push_front(linear_position[2]);
    }

    boost::circular_buffer<double> FrPositionRecorder::GetRecordOnDOF(unsigned int iDOF) const {
        return m_positions[iDOF];
    }

    void FrPositionRecorder::StepFinalize() {}

}  // end namespace frydom
