//
// Created by frongere on 01/10/18.
//

#include "FrInertia.h"

#include "FrVector.h"

#include "chrono/core/ChMatrix.h"

#include "chrono/core/ChMatrixDynamic.h"


namespace frydom {


    FrInertiaTensor_::FrInertiaTensor_() : m_mass(0.), m_cogPosition(0., 0., 0.) {}
//
//    FrInertiaTensor_::FrInertiaTensor_(double mass, double Ixx, double Iyy, double Izz, double Ixy, double Ixz,
//                                       double Iyz, const FrFrame_& coeffsFrame, const Position &cogPosition,
//                                       FRAME_CONVENTION fc) {
//
//        // On souhaite calculer et stocker l'inertier rIIg au centre de gravite exprime dans le repere local
//        // coeffsFrame donne le repere dans lequel sont exprimes les coeffs (position et orientation par rapport au
//        // repere local.
//        // On fournit la mass (invariante par rapport au repere) et la position du centre de gravite dans le repere local
//
//        // On a la formule :
//
//        // rIIg = rRp . pIIp . rRp^T  - rII(m, rOG - rOP)
//
//        // Avec :
//        // rIIg l'inertie en G exprimee dans le repere local
//        // rRp la matrice de rotation qui projette le repere p d'expression des coeffs dans le repere local
//        // rOG la position de G dans le repere local
//        // rOP la position de P, point d'expression des coeffs dans le repere local
//        // rII(m, rOG-rOP) l'inertie d'une masse m situee a une distance rPG = rOG - rOP exprimee dans le repere local
//
//
//
//
//
//
//
//
//
//    }


    FrInertiaTensor_::FrInertiaTensor_(double mass,
                                       double Ixx, double Iyy, double Izz,
                                       double Ixy, double Ixz, double Iyz,
                                       const FrFrame_& coeffsFrame,
                                       const Position& cogPosition,
                                       FRAME_CONVENTION fc) {

        if (IsNED(fc)) internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz); // Convert to NWU

        chrono::ChMatrix33<double> chInertia_pp = internal::BuildChInertiaMatrix(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);


        auto chRotation_rp = coeffsFrame.m_chronoFrame.Amatrix; // Allowed because FrInertiaTensor_ is a friend of FrFrame_
        // FIXME : faire gaffe a la convention adoptee pour decrire la rotation. Est-ce que la rotation donne bien
        // le repere de frame dans le repere local (R * vecteur exprime dans p donne vecteur exprime localement)...


        // Applying the generalized Huygens theorem to transport inertia at G, expressed in local frame coordinate system
        m_inertiaAtCOG.Reset();
        m_inertiaAtCOG.MatrMultiplyT(chRotation_rp * chInertia_pp, chRotation_rp);

        Position PG;
        PG = cogPosition - coeffsFrame.GetPosition(NWU);

        m_inertiaAtCOG -= internal::GetPointMassInertia(mass, PG);
        m_cogPosition = cogPosition;
        m_mass = mass;

    }












//    FrInertiaTensor_ MakePointInertiaTensor(double mass, const Position& position) {
//
//        double a, b, c, a2, b2, c2;
//        a = position[0];
//        b = position[1];
//        c = position[2];
//        a2 = a*a;
//        b2 = b*b;
//        c2 = c*c;
//
////        FrInertiaTensor_(mass,
////                b2+c2, a2+c2, a2+b2,
////                -a*b, -a*c, -b*c,
////                )
//
//
//    }


}  // end namespace frydom