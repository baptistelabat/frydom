//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBody.h"

#include <frydom/core/FrConstants.h>
#include <frydom/core/FrHydroBody.h>
#include "FrCurrentForce.h"
#include "FrCurrent.h"

#include "frydom/IO/FrLoader.h"

//#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {
namespace environment {

    FrCurrentForce::FrCurrentForce(std::string yaml_file) {
        coeffs_table = std::move(IO::MakeCurrentPolarCoeffTable(yaml_file));
    }


    environment::FrCurrent* FrCurrentForce::GetCurrentFlow() {
//        auto system = dynamic_cast<FrOffshoreSystem *>(GetBody()->GetSystem());  // Downcasting of a ChSystem
//        return system->GetCurrent();
    }

    void FrCurrentForce::UpdateState() {

        std::cout << "Updating current force" << std::endl;

        // Getting body's velocity with respect to water (including current effect)
        auto mybody = dynamic_cast<FrHydroBody*> (Body);
        auto relative_velocity = mybody->GetCurrentRelativeVelocity(NWU);
        relative_velocity.z() = 0.; // FIXME: doit-on le faire ?

        // Getting the angle between boat axis and velocity vector
        auto alpha = mybody->GetCurrentRelativeAngle(NED, DEG);
//        auto alpha = degrees(atan2(relative_velocity.y(), relative_velocity.x()));
        auto vel2 = relative_velocity.Length2();  // FIXME: ne prendre que la partie x, y, annuler le z...

        auto coeffs = coeffs_table.Eval(fabs(alpha));  // FIXME: plutot que prendre fabs(alpha), rendre plus intelligente la table (symmetrie)

        // Getting water density
        double rho_water = 1000.; // Aller chercher dans systeme !!
        double transverse_area = 10.; // Aller chercher dans hydro_body ?
        double lateral_area = 10.; // Aller chercher dans hydro_body ?
        double lpp = 50.;


        // FIXME: verifier ces valeurs !!!
        auto fx = - 0.5 * rho_water * coeffs["cx"] * transverse_area * vel2;
        auto fy = - 0.5 * rho_water * coeffs["cy"] * transverse_area * vel2;
        auto mz = - 0.5 * rho_water * coeffs["cz"] * lateral_area * vel2 * lpp;

        force.x() = fx;
        force.y() = fy;
        force.z() = 0.;
        moment.x() = 0.;
        moment.y() = 0.;
        moment.z() = mz;

    }



    }  // end namespace environment
}  // end namespace frydom
