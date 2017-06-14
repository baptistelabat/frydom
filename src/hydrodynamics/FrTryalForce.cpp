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
        relforce.x() = 1e6;  // force propulsive
        relforce.y() = 1e3;  // tel qu'on ait un couple cree...

        myfile.open("output.csv");

        myfile << "Time;X;Y;alpha;Vxrel;Vyrel;Vxabs;Vyabs;w;Fxrel;Fyrel;Fxabs;Fyabs;Mz" << std::endl;
    }

    void FrTryalForce::UpdateState() {

//        auto  var = Body->TransformDirectionParentToLocal(Body->GetPos_dt());
//        auto var2 = Body->TransformPointParentToLocal(Body->GetPos_dt());
//        auto rot = Body->GetRot();


//        if ((ChTime > 10) && (!done)){
//            relforce.y() = -relforce.y();
//            done = true;
//        }

//        force = Body->TransformDirectionParentToLocal(relforce);
        // On ajoute un amortissement en x et y
//        auto vel = Body->coord_dt.pos;
//        auto new_relforce = relforce;
//        double alpha_y = 1e3;
//
//        new_relforce.y() -= alpha_y * vel[1];


        // Transformation de la force dans le repere absolu
        force = Body->TransformDirectionLocalToParent(relforce);

        moment = vrelpoint.Cross(relforce);


        // On ajoute un amortissement en lacet
//        auto Wvelpar = Body->GetWvel_par();
//        double alpha = 1e6;
//        moment.z() -= alpha * Wvelpar.z();




//        moment.z() = 1e5;
//        moment.z() = 0;
//        std::cout << ChTime << std::endl;
//        std::cout << "Velocity: "<< "\t" << var.x() << "\t" << var.y() << "\t" << var.z() << std::endl;
//        std::cout << "force: "<< "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << std::endl;
//        std::cout << "relforce: "<< "\t" << relforce.x() << "\t" << relforce.y() << "\t" << relforce.z() << std::endl;
//        std::cout << "moment: "<< "\t" << moment.x() << "\t" << moment.y() << "\t" << moment.z() << std::endl << std::endl;


        // Writing the file

        auto pos = Body->coord.pos;
        auto rot = Body->coord.rot;
        auto vel = Body->coord_dt.pos;
        auto relvel = Body->TransformDirectionParentToLocal(vel);
        auto omega = Body->GetWvel_par();


        myfile << ChTime << ";"; // t
        myfile << pos[0] << ";"; // x
        myfile << pos[1] << ";"; // y
        myfile << rot[2] << ";"; // alpha
        myfile << relvel[0] << ";"; // Vxrel
        myfile << relvel[1] << ";"; // Vyrel
        myfile << vel[0] << ";"; // Vxabs
        myfile << vel[1] << ";"; // Vyabs
        myfile << omega[2] << ";"; // w
        myfile << relforce[0] << ";"; // Fxrel
        myfile << relforce[1] << ";"; // Fyrel
        myfile << force[0] << ";"; // Fxabs
        myfile << force[1] << ";"; // Fyabs
        myfile << moment[2] << ";"; // Mz
        myfile << std::endl;

    }

}
