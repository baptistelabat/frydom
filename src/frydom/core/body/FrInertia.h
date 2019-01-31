//
// Created by frongere on 01/10/18.
//

#ifndef FRYDOM_FRINERTIA_H
#define FRYDOM_FRINERTIA_H

//#include "chrono/core/ChMatrix33.h"

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrFrame.h"


namespace frydom {

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


        /// Set the mass, in kg
        void SetMass(double mass_kg);

        /// Set the cog position
        void SetCOGPosition(const Position& cogPosition, FRAME_CONVENTION fc);

        Matrix66<double> GetMatrix() const;


    private:

        friend std::ostream&operator<<(std::ostream& os, const FrInertiaTensor_& inertia);
        std::ostream& cout(std::ostream& os) const;

    };




    namespace internal {

        /// Swap the frame convention (NWU/NED) of inertia coefficients
        inline void SwapInertiaFrameConvention(double& Ixx, double& Iyy, double& Izz,
                                               double& Ixy, double& Ixz, double& Iyz) {
            Ixy = -Ixy;
            Ixz = -Ixz;
        }

    }  // end namespace internal


}  // end namespace frydom

#endif //FRYDOM_FRINERTIA_H
