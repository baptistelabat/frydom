//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRLINEARHYDROSTATICFORCE_H

// FIXME: attention, on a d'autres fichiers FrHydrostaticForce.h et .cpp
#include "chrono/physics/ChBody.h"
#include <frydom/core/FrForce.h>
#include <frydom/core/FrHydroBody.h>
#include "FrLinearHydrostaticStiffnessMatrix.h"


// FIXME: bien travailler sur le bon placement de la force hydrostatique lineaire !!!!

// TODO: Attention, il faut fonctionner avec une difference de position

namespace frydom {

    class FrLinearHydrostaticForce : public FrForce {

    private:
        FrLinearHydrostaticStiffnessMatrix m_stiffnessMatrix;

        // ##CC fix log
        double m_delta_z;
        double m_body_z;
        double m_eqframe_z;
        // ##CC

    public:
        FrLinearHydrostaticForce() {
            force.SetNull();
            moment.SetNull();
            m_log = hermes::Message("Fh","Hydrostatic force log");

        };

        FrLinearHydrostaticStiffnessMatrix* GetStiffnessMatrix() { return &m_stiffnessMatrix; }


        void UpdateState() override {
            // Getting the differential frame from

            auto eqFrame = dynamic_cast<FrHydroBody*>(Body)->GetEquilibriumFrame();
            auto bodyFrame = Body->GetFrame_REF_to_abs();
            //auto deltaFrame = bodyFrame >> eqFrame->GetInverse();
            auto deltaFrame = eqFrame->GetInverse() * bodyFrame;

            double heave = deltaFrame.GetPos().z();

            // ##CC fixe pour monitorer les variations de heave
            m_eqframe_z = eqFrame->GetPos().z();
            m_body_z = bodyFrame.GetPos().z();
            m_delta_z = heave;
            // ##CC

            chrono::ChVector<double> axis;
            double angle;
            internal::quat_to_axis_angle(deltaFrame.GetRot(), axis, angle, RAD);

            double roll = angle * axis.x();
            double pitch = angle * axis.y();

            chrono::ChVector<double> state(heave, roll, pitch);

            auto values = - (m_stiffnessMatrix * state);

            force.z() = values.x();
            // Cancelling gravity on body FIXME : pb de generalisation de la methode
            force -= Body->GetSystem()->Get_G_acc() * Body->GetMass();

            moment = Body->Dir_World2Body(chrono::ChVector<double> (values.y(), values.z(), 0.));
            // TODO: verifier que c'est la bonne fonction de transport
        }

        // ##CC Fix pour le log
        void InitializeLogs() override {

            m_log.AddField("eqFrame_z","m","Position of the equilibrium frame in z-direction",&m_eqframe_z);
            m_log.AddField("delta_z","m","Pertubation of the position in the z-direction", &m_delta_z);
            m_log.AddField("body_z","m","Position of the body in ref frame",&m_body_z);

            FrForce::InitializeLogs();
        }
        // ##CC

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "Fh_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRLINEARHYDROSTATICFORCE_H
