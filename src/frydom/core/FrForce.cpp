//
// Created by frongere on 08/06/17.
//

#include "chrono/physics/ChBody.h"

#include "FrForce.h"


namespace frydom{

//    void FrForce::GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const {
//        body_force = force;
//        body_torque = moment;
//    }




//    void FrForce::UpdateApplicationPoint() {
//
//        auto my_body = GetBody();
//        auto vmotion = chrono::VNULL;
//
//        if (move_x)
//            vmotion.x() = move_x->Get_y(ChTime);
//        if (move_y)
//            vmotion.y() = move_y->Get_y(ChTime);
//        if (move_z)
//            vmotion.z() = move_z->Get_y(ChTime);
//
//        switch (frame) {
//            case WORLD:
//                vpoint = Vadd(restpos, vmotion);                // Uw
//                vrelpoint = my_body->Point_World2Body(vpoint);  // Uo1 = [A]'(Uw-Xo1)
//                break;
//            case BODY:
//                vrelpoint = Vadd(restpos, vmotion);             // Uo1
//                vpoint = my_body->Point_Body2World(vrelpoint);  // Uw = Xo1+[A]Uo1
//                break;
//        }
//    }
//
//    void FrForce::UpdateChronoForce() {
//
//        double modforce;
//        auto my_body = GetBody();
//        auto vectforce = chrono::VNULL;
//        auto xyzforce = chrono::VNULL;
//
//        modforce = mforce * modula->Get_y(ChTime);
//
//        if (f_x)
//            xyzforce.x() = f_x->Get_y(ChTime);
//        if (f_y)
//            xyzforce.y() = f_y->Get_y(ChTime);
//        if (f_z)
//            xyzforce.z() = f_z->Get_y(ChTime);
//
//        switch (align) {
//            case WORLD_DIR:
//                vreldir = my_body->TransformDirectionParentToLocal(vdir);
//                vectforce = Vmul(vdir, modforce);
//                vectforce = Vadd(vectforce, xyzforce);
//                break;
//            case BODY_DIR:
//                vdir = my_body->TransformDirectionLocalToParent(vreldir);
//                vectforce = Vmul(vdir, modforce);
//                xyzforce = my_body->TransformDirectionLocalToParent(xyzforce);
//                vectforce = Vadd(vectforce, xyzforce);
//                break;
//        }
//
//        force += vectforce;                                           // Fw
//        relforce = my_body->TransformDirectionParentToLocal(force);  // Fo1 = [A]'Fw
//    }


}  // end namespace frydom