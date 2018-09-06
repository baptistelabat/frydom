//
// Created by frongere on 13/06/17.
//

#ifndef FRYDOM_FRITTC57_H
#define FRYDOM_FRITTC57_H

#include "frydom/core/FrForce.h"


namespace frydom {
    class FrITTC57 : public FrForce {

      private:
        double rho;  ///< Water density (kg/m**3)
        double nu;   ///< Kinematic viscosity of water (m**2/s)

        double Lpp;  ///< Characteristic length (length between perpendicular for ships) (meters)
        double k;    ///< Hull form factor
        double S;    ///< Hull wetted surface (m**2)
        double Ax;   ///< Hull frontal projected area (m**2)


      public:

        FrITTC57();

        void SetWaterDensity(double myrho) { rho = myrho; }
        double GetWaterDensity() { return rho; }

        void SetWaterKinematicViscosity(double mynu) { nu = mynu; }
        double GetWaterKinematicViscosity() { return nu; }

        void SetCharacteristicLength(double myLpp) { Lpp = myLpp; }
        double GetCharacteristicLength() { return Lpp; }

        void SetHullFormFactor(double myk) { k = myk; }
        double GetHullFormFactor() { return k; }

        void SetHullWettedSurface(double myS) { S = myS; }
        double GetHullWettedSurface() { return S; }

        void SetHullFrontalProjectedArea(double myAx) { Ax = myAx; }
        double GetHullFrontalProjectedArea() { return Ax; }

        void UpdateState() override;

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "Fittc57_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRITTC57_H
