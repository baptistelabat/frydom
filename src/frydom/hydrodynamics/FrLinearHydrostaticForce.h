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


            // ##CC debug hydrostatic
            std::cout << "eqFrame : " << std::endl;
            auto eq_x = eqFrame->GetA().Get_A_Xaxis();
            auto eq_y = eqFrame->GetA().Get_A_Yaxis();
            auto eq_z = eqFrame->GetA().Get_A_Zaxis();
            auto eq_pos = eqFrame->GetPos();
            std::cout << " pos    : " << eq_pos.x() << " ; " << eq_pos.y() << " ; " << eq_pos.z() << std::endl;
            std::cout << " x-axis : " << eq_x.x() << " ; " << eq_x.y() << " ; " << eq_x.z() << std::endl;
            std::cout << " y-axis : " << eq_y.x() << " ; " << eq_y.y() << " ; " << eq_y.z() << std::endl;
            std::cout << " z-axis : " << eq_z.x() << " ; " << eq_z.y() << " ; " << eq_z.z() << std::endl;

            std::cout << "bodyFrame : " << std::endl;
            auto eqB_x = bodyFrame.GetA().Get_A_Xaxis();
            auto eqB_y = bodyFrame.GetA().Get_A_Yaxis();
            auto eqB_z = bodyFrame.GetA().Get_A_Zaxis();
            auto eqB_pos = bodyFrame.GetPos();
            std::cout << " pos    : " << eqB_pos.x() << " ; " << eqB_pos.y() << " ; " << eqB_pos.z() << std::endl;
            std::cout << " x-axis : " << eqB_x.x() << " ; " << eqB_x.y() << " ; " << eqB_x.z() << std::endl;
            std::cout << " y-axis : " << eqB_y.x() << " ; " << eqB_y.y() << " ; " << eqB_y.z() << std::endl;
            std::cout << " z-axis : " << eqB_z.x() << " ; " << eqB_z.y() << " ; " << eqB_z.z() << std::endl;


            std::cout << "deltaFrame : " << std::endl;
            auto eqD_x = deltaFrame.GetA().Get_A_Xaxis();
            auto eqD_y = deltaFrame.GetA().Get_A_Yaxis();
            auto eqD_z = deltaFrame.GetA().Get_A_Zaxis();
            auto eqD_pos = deltaFrame.GetPos();
            std::cout << " pos    : " << eqD_pos.x() << " ; " << eqD_pos.y() << " ; " << eqD_pos.z() << std::endl;
            std::cout << " x-axis : " << eqD_x.x() << " ; " << eqD_x.y() << " ; " << eqD_x.z() << std::endl;
            std::cout << " y-axis : " << eqD_y.x() << " ; " << eqD_y.y() << " ; " << eqD_y.z() << std::endl;
            std::cout << " z-axis : " << eqD_z.x() << " ; " << eqD_z.y() << " ; " << eqD_z.z() << std::endl;


            std::cout << "Rotation : " << std::endl;
            std::cout << "angle (deg) : " << angle * RAD2DEG << std::endl;
            std::cout << "roll (deg) : " << roll * RAD2DEG << std::endl;
            std::cout << "pitch (deg) : " << pitch * RAD2DEG << std::endl;
            // ##CC
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
