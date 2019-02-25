//// ==========================================================================
//// FRyDoM - frydom-ce.org
////
//// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
//// All rights reserved.
////
//// Use of this source code is governed by a GPLv3 license that can be found
//// in the LICENSE file of FRyDoM.
////
//// ==========================================================================
//
//
//#ifndef FRYDOM_FRSTEADYPITCHTORQUE_H
//#define FRYDOM_FRSTEADYPITCHTORQUE_H
//
//// TODO : Plus utilisée dans le refactoring a supprimer
//
//#include "frydom/core/force/FrForce.h"
//
//namespace frydom {
//
//    /**
//     * \class FrSteadyPitchTorque
//     * \brief Class for computing a steady pitch torque.
//     */
//    class FrSteadyPitchTorque : public FrForce {
//
//    private:
//
//
//    public:
//
//
//        void UpdateState() override;
//
//        void SetLogPrefix(std::string prefix_name) override {
//            if (prefix_name=="") {
//                m_logPrefix = "Fuser_pitch_" + FrForce::m_logPrefix;
//            } else {
//                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
//            }
//        }
//
//    };
//
//    void FrSteadyPitchTorque::UpdateState() {
//
//        auto vx = Body->GetPos_dt().x();
//
//        double momentY;
//
//        momentY = 4.332 * std::pow(vx, 6)
//                - 9.1135 * std::pow(vx,5)
//                - 9.7756 * std::pow(vx,4)
//                + 34.232 * std::pow(vx,3)
//                - 22.7357 * std::pow(vx,2);
//
//        moment.y() = -momentY;
//    }
//
//
//
//}
//
//
//#endif //FRYDOM_FRSTEADYPITCHTORQUE_H
