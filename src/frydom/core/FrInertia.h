//
// Created by frongere on 01/10/18.
//

#ifndef FRYDOM_FRINERTIA_H
#define FRYDOM_FRINERTIA_H

//#include "chrono/core/ChMatrix33.h"

#include "FrVector.h"
#include "FrConvention.h"
#include "FrFrame.h"


namespace frydom {

    // TODO : degager cette classe et mettre directement en tant que methodes de FrBody_


    class InertiaMatrix : public mathutils::Matrix33<double> {

    public:

        InertiaMatrix() : mathutils::Matrix33<double>() {}

        // This constructor allows to construct Vector6d from Eigen expressions
        template <class OtherDerived>
        InertiaMatrix(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Matrix33<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        InertiaMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->mathutils::Matrix33<double>::operator=(other);
            return *this;
        }

        double GetIxx() const { return this->at(0, 0); }
        double GetIyy() const { return this->at(1, 1); }
        double GetIzz() const { return this->at(2, 2); }
        double GetIxy() const { return this->at(0, 1); }
        double GetIxz() const { return this->at(0, 2); }
        double GetIyz() const { return this->at(1, 2); }

    };


    /// Class to define the inertia tensor of a rigid body
    /// It stores the principal inertia parameters :
    ///     - mass
    ///     - local COG position expressed in the body reference frame
    ///     - inertia matrix expressed at COG in the body reference frame
    /// Internally, the frame convention used is NWU but it is still possible to get or set parameters in any frame
    /// convention
    class FrInertiaTensor_ {

        using InertiaMatrix = mathutils::Matrix33<double>;

    private:

        double m_mass;

        Position m_cogPosition;  ///< COG Position, stored internally in NWU convention

        InertiaMatrix m_inertiaAtCOG;  ///< Inertia matrix expressed at COG in reference frame

//        chrono::ChMatrix33<double> m_inertiaAtCOG;  ///< Inertia matrix

    public:

        // Dans inertia tensor, on stocke les coefficients de la matrice d'inertie exprimee au centre de gravite
        // dans le repere de reference dans lequel les coords du centre de gravite sont donnes

//      Chrono can't work with null inertia !!!
//        /// Default constructor with every inertia parameters set to null (COG position, mass, inertia matrix)
//        FrInertiaTensor_();
//
//        /// Constructor taking only a mass. COG Position and inertia matrix are set to null. Mass is in kg.
//        explicit FrInertiaTensor_(double mass);
//
//        /// Constructor taking only a  mass and a COG Position. The inertia matrix is set to null. Mass is in kg.
//        /// The frame convention holds on the COG Position.
//        FrInertiaTensor_(double mass, const Position& cogPosition, FRAME_CONVENTION fc);

        /// Constructor from standard inertia parameters. Inertia coefficients are expressed in coeffsFrame that can be
        /// different from the cogPosition. Both coeffsFrame and corPosition are relative to body reference coordinate
        /// system. Mass is in kg. The frame convention holds on inertia coefficients and COG Position.
        FrInertiaTensor_(double mass,
                         double Ixx, double Iyy, double Izz,
                         double Ixy, double Ixz, double Iyz,
                         const FrFrame_& coeffsFrame, const Position& cogPosition, FRAME_CONVENTION fc);

        /// Constructor from standard inertia parameters. Inertia coefficients are expressed at COG frame that is
        /// expressed relative to body reference coordinate system. Mass is in kg. The frame convention holds on
        /// inertia coefficients.
        FrInertiaTensor_(double mass,
                         double Ixx, double Iyy, double Izz,
                         double Ixy, double Ixz, double Iyz,
                         const FrFrame_& cogFrame, FRAME_CONVENTION fc);

        /// Get the mass in kg
        double GetMass() const;

        /// Get the COG position WRT body reference coordinate system
        const Position GetCOGPosition(FRAME_CONVENTION fc) const;

        /// Get the inertia coefficients in the body reference coordinate system and expressed at COG.
        void GetInertiaCoeffs(double& Ixx, double& Iyy, double& Izz,
                              double& Ixy, double& Ixz, double& Iyz,
                              FRAME_CONVENTION fc) const;

        /// Get the inertia matrix of a point mass
        static InertiaMatrix GetPointMassInertiaMatrix(double mass, const Position& PG);

    private:


        friend std::ostream&operator<<(std::ostream& os, const FrInertiaTensor_& inertia);
        std::ostream& cout(std::ostream& os) const;

    };


    namespace internal {
        /// Build the chrono inertia matrix from its 6 coefficients. Take care that the out of diagonal coefficients
        /// are directly those of the matrix (by eg, Ixy coefficient is -\int xy dV)
//        inline chrono::ChMatrix33<double> BuildChInertiaMatrix(double& Ixx, double& Iyy, double& Izz,
//                                                               double& Ixy, double& Ixz, double& Iyz) {
//            chrono::ChMatrix33<double> inertia;
//            inertia.Set33Element(0, 0, Ixx);
//            inertia.Set33Element(1, 1, Iyy);
//            inertia.Set33Element(2, 2, Izz);
//            inertia.Set33Element(0, 1, Ixy);
//            inertia.Set33Element(1, 0, Ixy);
//            inertia.Set33Element(0, 2, Ixz);
//            inertia.Set33Element(2, 0, Ixz);
//            inertia.Set33Element(1, 2, Iyz);
//            inertia.Set33Element(2, 1, Iyz);
//            return inertia;
//        }

        /// Swap the frame convention (NWU/NED) of inertia coefficients
        inline void SwapInertiaFrameConvention(double& Ixx, double& Iyy, double& Izz,
                                               double& Ixy, double& Ixz, double& Iyz) {
            Ixy = -Ixy;
            Ixz = -Ixz;
        }

//        /// Get the inertia matrix of a point mass
//        inline chrono::ChMatrix33<double> GetPointMassInertia(double mass, const Position& cogPos) {  // TODO : faire cette fonction plutot en Matrix33 !!
//            double a = cogPos[0];
//            double b = cogPos[1];
//            double c = cogPos[2];
//            double a2 = a*a;
//            double b2 = b*b;
//            double c2 = c*c;
//            double ab = a*b;
//            double ac = a*c;
//            double bc = b*c;
//            return chrono::ChMatrix33<double>(b2+c2, -ab,   -ac,
//                                              -ab  , a2+c2, -bc,
//                                              -ac  , -bc,   a2+b2) * mass;
//        }

//        /// Split the inertial matrix into individual inertia coefficients
//        inline void ChInertia2Coeffs(const chrono::ChMatrix33<double>& inertiaMat,
//                                     double& Ixx, double& Iyy, double& Izz,
//                                     double& Ixy, double& Ixz, double& Iyz) {
//
//            Ixx = inertiaMat.Get33Element(0, 0);
//            Iyy = inertiaMat.Get33Element(1, 1);
//            Izz = inertiaMat.Get33Element(2, 2);
//            Ixy = inertiaMat.Get33Element(0, 1);
//            Ixz = inertiaMat.Get33Element(0, 2);
//            Iyz = inertiaMat.Get33Element(1, 2);
//        }


    }  // end namespace internal

}  // end namespace frydom

#endif //FRYDOM_FRINERTIA_H
