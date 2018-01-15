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

            FrBody* m_body=nullptr;

            unsigned int m_size = 0;

            // Buffers
            std::vector<boost::circular_buffer<double>> m_velocities;

            double m_lastTime=-1;

    public:

        FrVelocityRecorder() = default;

        void SetSize(unsigned int size) { m_size = size; }

        unsigned int GetSize() const { return m_size; }

        void SetBody(FrBody* body) { m_body=body; }

        FrBody* GetBody() const { return m_body; }

        void Initialize() override {

            // Initializing every 6 recorders (6 DOF record) with size m_size (full buffer) and values 0.
            m_velocities.reserve(6);
            for (unsigned int i=0; i<6; i++) {
                m_velocities.emplace_back(
                        boost::circular_buffer<double>(m_size, m_size, 0.)
                );
            }
        }

        double GetTime() const { return m_lastTime; }

        void RecordVelocity() {

            auto currentTime = m_body->GetChTime();
            if (m_lastTime == currentTime) return;
            m_lastTime = currentTime;

            auto linear_velocity = m_body->GetPos_dt();
            auto angular_velocity = m_body->GetWvel_loc();
            angular_velocity = m_body->TransformDirectionLocalToParent(angular_velocity);


            // Note taht we use here push_front in order to get a conveniently organized  buffer
            // for the calculation of hydrodynamic radiation convolutions
            m_velocities[0].push_front(linear_velocity[0]);
            m_velocities[1].push_front(linear_velocity[1]);
            m_velocities[2].push_front(linear_velocity[2]);

            m_velocities[3].push_front(angular_velocity[0]);
            m_velocities[4].push_front(angular_velocity[1]);
            m_velocities[5].push_front(angular_velocity[2]);

        }


        boost::circular_buffer<double> GetRecordOnDOF(unsigned int iDOF) const {
            return m_velocities[iDOF];
        }

        void StepFinalize() override {}

    };

}  // end namespace frydom


#endif //FRYDOM_FRVELOCITYRECORDER_H
