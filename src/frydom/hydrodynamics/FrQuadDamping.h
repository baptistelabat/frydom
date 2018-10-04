//
// Created by Lucas Letournel on 11/09/18.
//

#ifndef FRYDOM_FRQUADDAMPING_H
#define FRYDOM_FRQUADDAMPING_H

#include "frydom/core/FrForce.h"

namespace frydom {

    class FrQuadDamping : public FrForce {
    private:
//        chrono::ChVector<double> m_translationDampings = chrono::VNULL;
//        chrono::ChVector<double> m_rotationDampings = chrono::VNULL;

        double m_translationDamping = 0;
        double m_rotationDamping = 0;

    public:

        FrQuadDamping() {};

        void SetTranslationalDamping(double B) {m_translationDamping = B;}
        void SetRotationalDamping(double B) {m_rotationDamping = B;}

        double GetTranslationalDamping() {return m_translationDamping;}
        double GetRotationalDamping() {return m_rotationDamping;}

//        void SetTranslationalDampings(const chrono::ChVector<double>& dampings) {
//            m_translationDampings = dampings;
//        }
//        void SetTranslationalDampings(double Bx, double By, double Bz) {
//            SetTranslationalDampings(chrono::ChVector<>(Bx,By,Bz));
//        }
//
//        void SetRotationalDampings(const chrono::ChVector<double>& dampings) {
//            m_rotationDampings = dampings;
//        }
//        void SetRotationalDampings(double Broll, double Bpitch, double Byaw) {
//            SetRotationalDampings(chrono::ChVector<>(Broll,Bpitch,Byaw));
//        }

        void UpdateState() override;

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "FLDamp" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }
    };

};

#endif //FRYDOM_FRQUADDAMPING_H
