//
// Created by frongere on 13/06/17.
//

#include <cmath>

#include "chrono/physics/ChBody.h"

#include "FrITTC57.h"



namespace frydom{

    FrITTC57::FrITTC57()
            : rho(1025.),
              nu(1e-6),
              Lpp(0.),
              k(0.25),
              S(0.),
              Ax(0.) {

    }

    void FrITTC57::UpdateState() {

        // Getting velocity
        auto abs_vel = Body->GetPos_dt();
        auto rel_vel = Body->TransformDirectionParentToLocal(abs_vel);

        // Getting relative velocity with respect to water
        auto ux = rel_vel.x();

        // Computing Reynolds number
        auto Re = std::abs(ux) * Lpp / nu;

        // Computing ITTC57 flat plate friction coefficient
        auto CF = 0.075 / pow( log10(Re)-2., 2. );

        // Residual friction
        auto CR = 0.12 * CF; // TODO: changer !! juste pour avoir qqch (mettre en attribut)

        // Total coefficient
        auto Ct = CF + CR;

        //
        relforce.x() = -0.5 * rho * S * (1+k) * Ct * ux * std::abs(ux);


        force = Body->TransformDirectionLocalToParent(relforce);

//        std::cout << force.x() << "\t" << force.y() << "\t" << force.z() << std::endl;

        auto force_verif = Body->TransformDirectionParentToLocal(force);


        std::cout << "Time: " << ChTime << "\tvel: " << ux << std::endl;
    }


}  // end namespace frydom