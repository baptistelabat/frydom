//
// Created by lucas on 01/02/19.
//

#ifndef FRYDOM_FRMOORINGBUOY_H
#define FRYDOM_FRMOORINGBUOY_H

#include "frydom/core/body/FrBody.h"
#include "frydom/core/force/FrForce.h"

namespace frydom {

    class FrLinearDamping_;

    class FrMooringBuoy : public FrBody_ {
    private:

        class FrSphereNonLinearHydrostaticForce : public FrForce_{
        public:
            void Update(double time) override;
            void StepFinalize() override {};
        };


        double m_radius = 1.;
        double c_volume;
        std::shared_ptr<FrSphereNonLinearHydrostaticForce> m_hydrostaticForce;
        std::shared_ptr<FrLinearDamping_> m_dampingForce;

    public:

        FrMooringBuoy(double radius, double mass, bool visual_asset = true, double damping=0);;

        double GetVolume() {return c_volume;}


        void Update() override;

    private:

        double computeDraft();

        double inline computeVolume(double Zt){
            c_volume = M_PI/3 * (Zt*(3*m_radius*m_radius - Zt*Zt) + 2*pow(m_radius,3));
        }




    };
} //end namespace frydom

#endif //FRYDOM_FRMOORINGBUOY_H
