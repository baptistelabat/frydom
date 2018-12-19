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







    // REFACTORING -------------->>>>>>>>>>


    // Forward Declaration
    class FrEnvironment_;

    class FrITTC57_ : public FrForce_ {

      private:

        double m_Lpp;  ///< Characteristic length (length between perpendicular for ships) (meters)
        double m_k;    ///< Hull form factor
        double m_S;    ///< Hull wetted surface (m**2)
        double m_Ax;   ///< Hull frontal projected area (m**2)

        FrEnvironment_* m_environment;

      public:

//        FrITTC57();

        FrITTC57_(double characteristicLength, double hullFormFactor, double hullWetSurface, double hullFrontalArea);

//        void SetWaterDensity(double myrho);
//
//        double GetWaterDensity();
//
//        void SetWaterKinematicViscosity(double mynu);
//
//        double GetWaterKinematicViscosity();
//
//        void SetCharacteristicLength(double myLpp);
//
//        double GetCharacteristicLength();
//
//        void SetHullFormFactor(double myk);
//
//        double GetHullFormFactor();
//
//        void SetHullWettedSurface(double myS);
//
//        double GetHullWettedSurface();
//
//        void SetHullFrontalProjectedArea(double myAx);
//
//        double GetHullFrontalProjectedArea();

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

//        void SetLogPrefix(std::string prefix_name) override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRITTC57_H
