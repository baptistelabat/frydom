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

    public:
        FrLinearHydrostaticForce() {
            force.SetNull();
            moment.SetNull();
        };

        FrLinearHydrostaticStiffnessMatrix* GetStiffnessMatrix() { return &m_stiffnessMatrix; }


        void UpdateState() override {
            // Getting the differential frame from

            auto eqFrame = dynamic_cast<FrHydroBody*>(Body)->GetEquilibriumFrame();
            auto bodyFrame = Body->GetFrame_REF_to_abs();
            auto deltaFrame = eqFrame.GetInverse() * bodyFrame;

            double heave = deltaFrame.GetPos().z();

            chrono::ChVector<double> axis;
            double angle;
            quat_to_axis_angle(deltaFrame.GetRot(), axis, angle, RAD);

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


    };

}  // end namespace frydom


#endif //FRYDOM_FRLINEARHYDROSTATICFORCE_H
