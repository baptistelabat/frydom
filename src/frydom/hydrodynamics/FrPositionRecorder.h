//
// Created by camille on 08/06/18.
//

#ifndef FRYDOM_FRPOSITIONRECORDER_H
#define FRYDOM_FRPOSITIONRECORDER_H

#include "boost/circular_buffer.hpp"
#include "chrono/physics/ChProbe.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {

    class FrPositionRecorder : public chrono::ChProbe, public FrObject {

    private:
        FrBody* m_body = nullptr;           ///< Body to which the position is recrorded
        unsigned int m_size = 0;            ///< size of the recorder
        std::vector<boost::circular_buffer<double>> m_positions;    ///< Recorded position
        double m_lastTime = -1;             ///< Time of the last element store in the recorder

    public:
        /// Default constructor
        FrPositionRecorder() = default;

        /// Set size of the recorder
        void SetSize(unsigned int size) { m_size = size; }

        /// Return the size of the recorder
        unsigned int GetSize() const { return m_size; }

        /// Set the body object linked to the recorder
        void SetBody(FrBody* body) { m_body = body; }

        /// Return the body object linked to the recorder
        FrBody* GetBody() const { return m_body; }

        /// Initialization step
        void Initialize() override {
            m_positions.reserve(3);
            for (unsigned int i=0; i<3; i++) {
                m_positions.emplace_back(boost::circular_buffer<double>(m_size, m_size, 0.));
            }
        }

        /// Return last time saved in the recorder
        double GetTime() const { return m_lastTime; }

        /// Save current velocity into recorder
        void RecordPosition() {

            auto currentTime = m_body->GetChTime();
            if (m_lastTime == currentTime) return;
            m_lastTime = currentTime;

            auto linear_position = m_body->GetPos();

            m_positions[0].push_front(linear_position[0]);
            m_positions[1].push_front(linear_position[1]);
            m_positions[2].push_front(linear_position[2]);
        }

        /// Return the velocity recorded for specific dof
        boost::circular_buffer<double> GetRecordOnDOF(unsigned int iDOF) const {
            return m_positions[iDOF];
        }

        /// Method executed at the end of each time step
        void StepFinalize() override {}
    };

}   // end namespace frydom



#endif //FRYDOM_FRPOSITIONRECORDER_H
