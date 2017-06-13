//
// Created by frongere on 07/06/17.
//

#include "FrTryalForce.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"




namespace frydom {

    FrTryalForce::FrTryalForce()
            : FrForce(),
              done(false){

        vrelpoint.x() = -40;  // position du point d'application
        relforce.x() = 1e6;  // force propulsive
        relforce.y() = 1e3;  // tel qu'on ait un couple cree...
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
        force = Body->TransformDirectionLocalToParent(relforce);

        moment = vrelpoint.Cross(relforce);

//        moment.z() = 1e5;
//        moment.z() = 0;
//        std::cout << ChTime << std::endl;
//        std::cout << "Velocity: "<< "\t" << var.x() << "\t" << var.y() << "\t" << var.z() << std::endl;
//        std::cout << "force: "<< "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << std::endl;
//        std::cout << "relforce: "<< "\t" << relforce.x() << "\t" << relforce.y() << "\t" << relforce.z() << std::endl;
//        std::cout << "moment: "<< "\t" << moment.x() << "\t" << moment.y() << "\t" << moment.z() << std::endl << std::endl;

    }

}
