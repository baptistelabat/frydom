//
// Created by frongere on 07/06/17.
//

#include "FrTryalForce.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"


// TODO: plutot definir une direction et une intensite definis dans le repere local
// du corps.

namespace frydom {

    FrTryalForce::FrTryalForce()
            : FrForce(),
              done(false) {

        vrelpoint.x() = -40;  // position du point d'application
        vrelpoint.z() = 0;
        relforce.x() = 1e8;  // force propulsive
        relforce.y() = 0;  // tel qu'on ait un couple cree...

//        myfile.open("output.csv");
//
//        myfile << "Time;X;Y;alpha;Vxrel;Vyrel;Vxabs;Vyabs;w;Fxrel;Fyrel;Fxabs;Fyabs;Mz" << std::endl;
    }

    void FrTryalForce::UpdateState() {


        // Transformation de la force dans le repere absolu
        force = Body->TransformDirectionLocalToParent(relforce);

        moment = vrelpoint.Cross(relforce);


//        // Writing the file
//
//        auto pos = Body->coord.pos;
//        auto rot = Body->coord.rot;
//        auto vel = Body->coord_dt.pos;
//        auto relvel = Body->TransformDirectionParentToLocal(vel);
//        auto omega = Body->GetWvel_par();
//
//
//        myfile << ChTime << ";"; // t
//        myfile << pos[0] << ";"; // x
//        myfile << pos[1] << ";"; // y
//        myfile << rot[2] << ";"; // alpha
//        myfile << relvel[0] << ";"; // Vxrel
//        myfile << relvel[1] << ";"; // Vyrel
//        myfile << vel[0] << ";"; // Vxabs
//        myfile << vel[1] << ";"; // Vyabs
//        myfile << omega[2] << ";"; // w
//        myfile << relforce[0] << ";"; // Fxrel
//        myfile << relforce[1] << ";"; // Fyrel
//        myfile << force[0] << ";"; // Fxabs
//        myfile << force[1] << ";"; // Fyabs
//        myfile << moment[2] << ";"; // Mz
//        myfile << std::endl;

    }

}
