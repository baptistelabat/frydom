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


#ifndef FRYDOM_FRVELOCITYRECORDER_H
#define FRYDOM_FRVELOCITYRECORDER_H

#include "boost/circular_buffer.hpp"

#include "chrono/physics/ChProbe.h"

#include "frydom/core/common/FrObject.h"
#include "frydom/core/junk/FrHydroBody.h"
#include "frydom/core/FrOffshoreSystem.h"


namespace frydom {

    /**
     * \class FrVelocityRecorder
     * \brief Class not used.
     */
    class FrVelocityRecorder : public chrono::ChProbe, public FrObject {  // FIXME : pas de derivation directe d'objets chrono !!!!

    private:

            FrBody* m_body=nullptr;

    protected:

            unsigned int m_size = 0;
            unsigned int m_nstep = 0;

            // Buffers
            std::vector<boost::circular_buffer<double>> m_velocities;

            double m_lastTime=-1;

    public:

        FrVelocityRecorder() = default;  // TODO : Retirer, on doit toujours instancier avec un corps

        explicit FrVelocityRecorder(FrBody* body);

        void SetSize(unsigned int size);

        unsigned int GetSize() const;

        unsigned int GetNStep() const;

        void SetBody(FrBody* body);  // TODO : retirer, seulement le constructeur

        FrBody* GetBody() const;

        virtual void Initialize() override;

        double GetTime() const;

        /// Return the linear velocity of the body in global frame (m/s)
        virtual chrono::ChVector<double> GetLinearVelocity() const;

        /// Return the angular velocity of the body
        virtual chrono::ChVector<double> GetAngularVelocity() const;

        virtual void RecordVelocity();

        boost::circular_buffer<double> GetRecordOnDOF(unsigned int iDOF) const;

        virtual void StepFinalize() override;

    };



    /**
     * \class FrPerturbationVelocityRecorder
     * \brief Class not used.
     */
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
