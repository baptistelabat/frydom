//
// Created by frongere on 11/09/17.
//

#ifndef FRYDOM_FRLINEARDAMPING_H
#define FRYDOM_FRLINEARDAMPING_H

#endif //FRYDOM_FRLINEARDAMPING_H

#include "frydom/core/FrForce.h"

namespace frydom {

    class FrLinearDamping : public FrForce {

    private:
        chrono::ChVector<double> m_maneuveuringDampings = chrono::VNULL;
        chrono::ChVector<double> m_seakeepingDampings = chrono::VNULL;

    public:

        FrLinearDamping() {};

//        FrLinearDamping(const double Dx, const double Dy, const double Dwz) : m_Dx(Dx), m_Dy(Dy), m_Dwz(Dwz){};

        void SetManeuveuringDampings(const chrono::ChVector<double>& dampings) {
            m_maneuveuringDampings = dampings;
        }

        void SetManeuveuringDampings(double Bx, double By, double Byaw) {
            SetManeuveuringDampings(chrono::ChVector<double>(Bx, By, Byaw));
        }

        void SetSeakeepingDampings(const chrono::ChVector<double>& dampings) {
            m_seakeepingDampings = dampings;
        }

        void SetSeakeepingDampings(double Bz, double Broll, double Bpitch) {
            SetSeakeepingDampings(chrono::ChVector<double>(Bz, Broll, Bpitch));
        }

        void SetTranslationalDampings(const chrono::ChVector<double>& dampings) {
            m_maneuveuringDampings.x() = dampings.x();
            m_maneuveuringDampings.y() = dampings.y();
            m_seakeepingDampings.x() = dampings.z();
        }

        void SetTranslationalDampings(double Bx, double By, double Bz) {
            SetTranslationalDampings(chrono::ChVector<double>(Bx, By, Bz));
        }

        void SetRotationalDampings(const chrono::ChVector<double>& dampings) {
            m_maneuveuringDampings.z() = dampings.z();
            m_seakeepingDampings.y() = dampings.x();
            m_seakeepingDampings.z() = dampings.y();
        }

        void SetRotationalDampings(double Broll, double Bpitch, double Byaw) {
            SetRotationalDampings(chrono::ChVector<double>(Broll, Bpitch, Byaw));
        }

        void UpdateState() override;

    };


};  // end namespace frydom