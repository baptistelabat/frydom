//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRVELOCITYRECORDER_H
#define FRYDOM_FRVELOCITYRECORDER_H

#include "boost/circular_buffer.hpp"

#include "chrono/physics/ChProbe.h"

#include "frydom/core/FrObject.h"
#include "frydom/core/FrBody.h"
#include "frydom/core/FrOffshoreSystem.h"


namespace frydom {

    class FrVelocityRecorder : public chrono::ChProbe, public FrObject {
        private:
            chrono::ChSystem* m_system = nullptr;

            std::shared_ptr<FrBody> m_body;

            unsigned int m_size;

            // Buffers
            boost::circular_buffer<double> m_vx;
            boost::circular_buffer<double> m_vy;
            boost::circular_buffer<double> m_vz;
            boost::circular_buffer<double> m_wx;
            boost::circular_buffer<double> m_wy;
            boost::circular_buffer<double> m_wz;

    public:

        FrVelocityRecorder() {};

        void SetSystem(FrOffshoreSystem* system) { m_system = system; }

        void SetSize(unsigned int size) { m_size = size; }

        unsigned int GetSize() const { return m_size; }

        void Initialize() override {
            // Initializing every 6 recorders with size m_size (full buffer) and values 0.
            m_vx = boost::circular_buffer<double>(m_size, m_size, 0.);
            m_vy = boost::circular_buffer<double>(m_size, m_size, 0.);
            m_vz = boost::circular_buffer<double>(m_size, m_size, 0.);
            m_wx = boost::circular_buffer<double>(m_size, m_size, 0.);
            m_wy = boost::circular_buffer<double>(m_size, m_size, 0.);
            m_wz = boost::circular_buffer<double>(m_size, m_size, 0.);
        }

        void StepFinalize() {}

    };

}  // end namespace frydom


#endif //FRYDOM_FRVELOCITYRECORDER_H
