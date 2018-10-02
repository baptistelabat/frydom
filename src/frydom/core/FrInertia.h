//
// Created by frongere on 01/10/18.
//

#ifndef FRYDOM_FRINERTIA_H
#define FRYDOM_FRINERTIA_H

#include "chrono/core/ChMatrix33.h"

#include "FrVector.h"
#include "FrGeographic.h"
#include "FrFrame.h"


namespace frydom {

    // TODO : degager cette classe et mettre directement en tant que methodes de FrBody_


    class FrInertiaTensor_ {

    private:

        double m_mass;

        Position m_cogPosition;

        chrono::ChMatrix33<double> m_inertiaAtCOG;

    public:

        // Dans inertia tensor, on stocke les coefficients de la matrice d'inertie exprimee au centre de gravite
        // dans le repere de reference dans lequel les coords du centre de gravite sont donnes

        FrInertiaTensor_();

        FrInertiaTensor_(double mass,
                         double Ixx, double Iyy, double Izz,
                         double Ixy, double Ixz, double Iyz,
                         const FrFrame_& coeffsFrame, const Position& cogPosition, FRAME_CONVENTION fc);

//        FrInertiaTensor_(double mass,
//                         double Ixx, double Iyy, double Izz,
//                         double Ixy, double Ixz, double Iyz,
//                         const FrFrame_& cogFrame, FRAME_CONVENTION fc);




//        void SetCOGPosition(const Position& cogPosition, bool transportInertia);
//
//        void SetInertiaCoeffs(double Ixx, double Iyy, double Izz,
//                              double Ixy, double Ixz, double Iyz,
//                              const Position& coeffsPosition);


    };


    namespace internal {

        inline chrono::ChMatrix33<double> BuildChInertiaMatrix(double& Ixx, double& Iyy, double& Izz,
                                                               double& Ixy, double& Ixz, double& Iyz) {
            chrono::ChMatrix33<double> inertia;
            inertia.Set33Element(0, 0, Ixx);
            inertia.Set33Element(1, 1, Iyy);
            inertia.Set33Element(2, 2, Izz);
            inertia.Set33Element(0, 1, Ixy);
            inertia.Set33Element(1, 0, Ixy);
            inertia.Set33Element(0, 2, Ixz);
            inertia.Set33Element(2, 0, Ixz);
            inertia.Set33Element(1, 2, Iyz);
            inertia.Set33Element(2, 1, Iyz);
            return inertia;
        }

        inline void SwapInertiaFrameConvention(double& Ixx, double& Iyy, double& Izz,
                                               double& Ixy, double& Ixz, double& Iyz) {
            Ixy = -Ixy;
            Ixz = -Ixz;
        }

        inline chrono::ChMatrix33<double> GetPointMassInertia(double mass, const Position& cogPos) {
            double a = cogPos[0];
            double b = cogPos[1];
            double c = cogPos[2];
            double a2 = a*a;
            double b2 = b*b;
            double c2 = c*c;
            double ab = a*b;
            double ac = a*c;
            double bc = b*c;
            return chrono::ChMatrix33<double>(b2+c2, -ab,   -ac,
                                              -ab  , a2+c2, -bc,
                                              -ac  , -bc,   a2+b2) * mass;
        }


    }  // end namespace internal



}  // end namespace frydom

#endif //FRYDOM_FRINERTIA_H
