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

        void SetManeuveuringDampings(const chrono::ChVector<double> dampings) {
            m_maneuveuringDampings = dampings;
        }

        void SetSeakeepingDampings(const chrono::ChVector<double> dampings) {
            m_seakeepingDampings = dampings;
        }

        void UpdateState() override;

    };


};  // end namespace frydom