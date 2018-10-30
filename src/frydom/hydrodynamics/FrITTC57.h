//
// Created by frongere on 13/06/17.
//

#ifndef FRYDOM_FRITTC57_H
#define FRYDOM_FRITTC57_H

#include "frydom/core/FrForce.h"


namespace frydom {
    class FrITTC57 : public FrForce {

      private:
        double rho;  ///< Water density (kg/m**3)  // TODO : aller chercher dans environment
        double nu;   ///< Kinematic viscosity of water (m**2/s) // TODO : aller chercher dans environment

        double Lpp;  ///< Characteristic length (length between perpendicular for ships) (meters)
        double k;    ///< Hull form factor
        double S;    ///< Hull wetted surface (m**2)
        double Ax;   ///< Hull frontal projected area (m**2)


      public:

        FrITTC57();

        void SetWaterDensity(double myrho);

        double GetWaterDensity();

        void SetWaterKinematicViscosity(double mynu);

        double GetWaterKinematicViscosity();

        void SetCharacteristicLength(double myLpp);

        double GetCharacteristicLength();

        void SetHullFormFactor(double myk);

        double GetHullFormFactor();

        void SetHullWettedSurface(double myS);

        double GetHullWettedSurface();

        void SetHullFrontalProjectedArea(double myAx);

        double GetHullFrontalProjectedArea();

        void UpdateState() override;

        void SetLogPrefix(std::string prefix_name) override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRITTC57_H
