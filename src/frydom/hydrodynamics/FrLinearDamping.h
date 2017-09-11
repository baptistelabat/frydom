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
        double m_Dx = 0.;
        double m_Dy = 0.;
        double m_Dwz = 0.;


    public:

        FrLinearDamping() : m_Dx(0.), m_Dy(0.), m_Dwz(0.) {};

        FrLinearDamping(const double Dx, const double Dy, const double Dwz) : m_Dx(Dx), m_Dy(Dy), m_Dwz(Dwz){};


        void UpdateState() override {



        }


    };


};  // end namespace frydom