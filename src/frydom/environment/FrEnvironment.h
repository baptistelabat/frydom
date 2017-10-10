//
// Created by frongere on 10/07/17.
//

#ifndef FRYDOM_FRENVIRONMENT_H
#define FRYDOM_FRENVIRONMENT_H


// Current includes

#include "current/FrCurrent.h"
#include "current/FrCurrentPolarCoeffs.h"
#include "current/FrCurrentForce.h"

// Waves includes
#include "waves/FrFlatFreeSurface.h"

#include "tidal/FrTidalModel.h"

// Wind includes
#include "wind/FrWind.h"
#include "wind/FrWindForce.h"

// Seabed includes
#include "seabed/FrSeabed.h"


namespace frydom {
namespace environment {
    /// Class to store the different elements composing the offshore environment
    class FrEnvironment {

    private:

        std::unique_ptr<FrFreeSurface> m_free_surface;
        std::unique_ptr<FrTidal> m_tidal;
        std::unique_ptr<FrCurrent> m_current;
        std::unique_ptr<FrWind> m_wind;
        std::unique_ptr<FrSeabed> m_seabed;

        double m_water_density = 1025.;
        double m_air_density = 1.204;
        double m_gravity_acceleration = 9.81;

        double m_sea_temperature = 15.;
        double m_air_temperature = 20.;

        double m_water_kinematic_viscosity;

        double m_atmospheric_pressure;


    public:
        FrEnvironment() {

            m_free_surface = std::make_unique<FrFlatFreeSurface>();
            m_tidal = std::make_unique<FrTidal>();
            m_current = std::make_unique<FrCurrent>();
            m_wind = std::make_unique<FrWind>();
            m_seabed = std::make_unique<FrSeabed>();

        }

        FrFreeSurface* GetFreeSurface() const { return m_free_surface.get(); }
        FrTidal* GetTidal() const { return m_tidal.get(); }
        FrCurrent* GetCurrent() const { return m_current.get(); }
        FrWind* GetWind() const { return m_wind.get(); }
        FrSeabed* GetSeabed() const { return m_seabed.get(); }







    friend class FrOffshoreSystem;

    };

}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRENVIRONMENT_H
