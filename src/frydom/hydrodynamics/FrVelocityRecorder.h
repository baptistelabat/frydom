//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRVELOCITYRECORDER_H
#define FRYDOM_FRVELOCITYRECORDER_H

#include "boost/circular_buffer.hpp"

#include "chrono/physics/ChProbe.h"

#include "frydom/core/FrObject.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/core/FrOffshoreSystem.h"


namespace frydom {

    class FrVelocityRecorder : public chrono::ChProbe, public FrObject {

    private:

            FrBody* m_body=nullptr;

    protected:

            unsigned int m_size = 0;
            unsigned int m_nstep = 0;

            // Buffers
            std::vector<boost::circular_buffer<double>> m_velocities;

            double m_lastTime=-1;

    public:

        FrVelocityRecorder() = default;

        void SetSize(unsigned int size) { m_size = size; }

        unsigned int GetSize() const { return m_size; }

        unsigned int GetNStep() const { return std::min(m_nstep, m_size); }

        void SetBody(FrBody* body) { m_body=body; }

        FrBody* GetBody() const { return m_body; }

        virtual void Initialize() override {

            // Initializing every 6 recorders (6 DOF record) with size m_size (full buffer) and values 0.
            m_velocities.reserve(6);
            for (unsigned int i=0; i<6; i++) {
                m_velocities.emplace_back(
                        boost::circular_buffer<double>(m_size, m_size, 0.)
                );
            }
        }

        double GetTime() const { return m_lastTime; }

        /// Return the linear velocity of the body in global frame (m/s)
        virtual chrono::ChVector<double> GetLinearVelocity() const {
            return m_body->GetPos_dt();
        }

        /// Return the angular velocity of the body
        virtual chrono::ChVector<double> GetAngularVelocity() const {
            auto angular_velocity = m_body->GetWvel_loc();
            return m_body->TransformDirectionLocalToParent(angular_velocity);
        }

        virtual void RecordVelocity() {

            auto currentTime = m_body->GetChTime();
            if (m_lastTime == currentTime) return;
            m_lastTime = currentTime;

            auto linear_velocity = GetLinearVelocity();
            auto angular_velocity = GetAngularVelocity();

            // Note taht we use here push_front in order to get a conveniently organized  buffer
            // for the calculation of hydrodynamic radiation convolutions
            m_velocities[0].push_front(linear_velocity[0]);
            m_velocities[1].push_front(linear_velocity[1]);
            m_velocities[2].push_front(linear_velocity[2]);

            m_velocities[3].push_front(angular_velocity[0]);
            m_velocities[4].push_front(angular_velocity[1]);
            m_velocities[5].push_front(angular_velocity[2]);

            m_nstep += 1;

        }

        boost::circular_buffer<double> GetRecordOnDOF(unsigned int iDOF) const {
            return m_velocities[iDOF];
        }

        virtual void StepFinalize() override {}

    };



    class FrPerturbationVelocityRecorder : public FrVelocityRecorder {

    protected:
        FrHydroBody* m_body;        ///< Body to which the velocity recorder is applied

    public:
        /// Default constructor
        FrPerturbationVelocityRecorder() = default;

        /// Return the hydro body to which the velocity recorder is applied
        FrHydroBody* GetBody() const { return m_body; }

        /// Define the hydro body to which the velocity recorder is applied
        void SetBody(FrHydroBody* body) { m_body = body; }

        /// Return the perturbation linear velocity of the body in the equilibrium frame
        chrono::ChVector<double> GetLinearVelocity() const override {
            return m_body->GetLinearVelocityPert();
        }

        /// Return the perturbation angular velocity of the body in the equilibrium frame
        chrono::ChVector<double> GetAngularVelocity() const override {
            auto angular_velocity = m_body->GetAngularVelocityPert();
            return m_body->TransformDirectionLocalToParent(angular_velocity);
        }

        void RecordVelocity() override {

            auto currentTime = m_body->GetChTime();
            if (m_lastTime == currentTime) return;
            m_lastTime = currentTime;

            auto linear_velocity = GetLinearVelocity();
            auto angular_velocity = GetAngularVelocity();

            // Note taht we use here push_front in order to get a conveniently organized  buffer
            // for the calculation of hydrodynamic radiation convolutions
            m_velocities[0].push_front(linear_velocity[0]);
            m_velocities[1].push_front(linear_velocity[1]);
            m_velocities[2].push_front(linear_velocity[2]);

            m_velocities[3].push_front(angular_velocity[0]);
            m_velocities[4].push_front(angular_velocity[1]);
            m_velocities[5].push_front(angular_velocity[2]);

        }
    };

}  // end namespace frydom


#endif //FRYDOM_FRVELOCITYRECORDER_H
