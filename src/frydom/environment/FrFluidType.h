//
// Created by frongere on 31/10/18.
//

#ifndef FRYDOM_FRFLUIDTYPE_H
#define FRYDOM_FRFLUIDTYPE_H


#include "frydom/core/FrUnits.h"

namespace frydom {


    enum FLUID_TYPE {
        AIR,
        WATER
    };

    struct FrFluidProperties{
        // Fluid properties:
        /// Fluid temperature, in Celsius (SI:Kelvin?)
        double m_temperature;
        /// Fluid density, in kg/m³
        double m_density;
        /// Fluid dynamic viscosity, in (Ns/m² = Pa.s)
        double m_dynamicViscosity;
        /// Fluid kinematic viscosity, in ? (m²/s)
        double m_kinematicViscosity;
        /// Fluid salinity, dimensionless (g/kg)
        double m_salinity; //TODO : ITTC Recommended Procedures : Fresh Water and Seawater Properties
        /// Fluid static pressure (MPa)
        double m_pressure;
    };


    class FrFluidProperties{
    private:

        // Fluid properties:
        /// Fluid temperature, in Celsius (SI:Kelvin?)
        double m_temperature;
        /// Fluid density, in kg/m³
        double m_density;
        /// Fluid dynamic viscosity, in (Ns/m² = Pa.s)
        double m_dynamicViscosity;
        /// Fluid kinematic viscosity, in ? (m²/s)
        double m_kinematicViscosity;
        /// Fluid salinity, dimensionless (g/kg)
        double m_salinity; //TODO : ITTC Recommended Procedures : Fresh Water and Seawater Properties
        /// Fluid static pressure (MPa)
        double m_pressure;

    public:

        FrFluidProperties() {
            m_temperature = 0.;
            m_density = 0.;
            m_dynamicViscosity = 0.;
            m_kinematicViscosity = 0.;
            m_salinity = 0.;
            m_pressure = 0.;
        }
        FrFluidProperties(double Temperature, double Density, double DynamicViscosity, double KinematicViscosity,
                double Salinity, double Pressure) {
            m_temperature = Temperature;
            m_density = Density;
            m_dynamicViscosity = DynamicViscosity;
            m_kinematicViscosity = KinematicViscosity;
            m_salinity = Salinity;
            m_pressure = Pressure;
        }



    };

//    // dans environnement, on aura Atmosphere et Sea en amont de current et wind ??
//
//
//
//
//    // TODO : Ici, on va definir des classes Air et Water pour stocker les pptes des fluides consideres
//
//    class Air {
//
//    private:
//
//        double m_temperatureCelsius = 15.;
//
//        double m_dynamicViscosity;
//
//
//
//
//
//
//    };
//
//    class Water {
//        temp
//        salinity
//        dynamicviscosity
//
//        kinematicviscosity en cache
//
//
//
//    };

}

#endif //FRYDOM_FRFLUIDTYPE_H
