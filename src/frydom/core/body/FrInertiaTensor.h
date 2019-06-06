// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRINERTIATENSOR_H
#define FRYDOM_FRINERTIATENSOR_H

#include <iostream>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrFrame.h"



namespace frydom {

    /**
     * \class FrInertiaTensor
     * \brief Class for defining the intertia tensor of a rigid body.
     *
     * It stores the principal inertia parameters :
     *     - mass
     *     - local COG position expressed in the body reference frame
     *     - inertia matrix expressed at COG in the body reference frame
     *
     * Internally, the frame convention used is NWU but it is still possible to get or set parameters in any frame
     * convention
     */
    class FrInertiaTensor {

        using InertiaMatrix = mathutils::Matrix33<double>;

    private:

        double m_mass;

        Position m_cogPosition;  ///< COG Position, stored internally in NWU convention

        InertiaMatrix m_inertiaAtCOG;  ///< Inertia matrix expressed at COG in reference frame

    public:

        /// Constructor from standard inertia parameters. Inertia coefficients are expressed in coeffsFrame that can be
        /// different from the cogPosition. Both coeffsFrame and cogPosition are relative to body reference coordinate
        /// system. Mass is in kg. The frame convention holds on inertia coefficients and COG Position.
        FrInertiaTensor(double mass,
                         double Ixx, double Iyy, double Izz,
                         double Ixy, double Ixz, double Iyz,
                         const FrFrame& coeffsFrame, const Position& cogPosition, FRAME_CONVENTION fc);

        /// Constructor from standard inertia parameters. Inertia coefficients are expressed at COG frame that is
        /// expressed relative to body reference coordinate system. Mass is in kg. The frame convention holds on
        /// inertia coefficients.
        FrInertiaTensor(double mass,
                         double Ixx, double Iyy, double Izz,
                         double Ixy, double Ixz, double Iyz,
                         const Position& cogPos, FRAME_CONVENTION fc);

        /// Get the mass in kg
        double GetMass() const;

        /// Get the COG position WRT body reference coordinate system
        const Position GetCOGPosition(FRAME_CONVENTION fc) const;

        /// Get the inertia coefficients in the body reference coordinate system and expressed at COG.
        void GetInertiaCoeffsAtCOG(double &Ixx, double &Iyy, double &Izz,
                                   double &Ixy, double &Ixz, double &Iyz,
                                   FRAME_CONVENTION fc) const;

        /// Get the inertia coefficients in the body reference coordinate system and expressed at a specified reference
        /// frame, relative to the body reference frame.
        void GetInertiaCoeffsAtFrame(double &Ixx, double &Iyy, double &Izz,
                                   double &Ixy, double &Ixz, double &Iyz,
                                   const FrFrame& frame,
                                   FRAME_CONVENTION fc) const;

        /// Get the inertia matrix of a point mass
        static InertiaMatrix GetPointMassInertiaMatrix(double mass, const Position& PG);

        Matrix66<double> GetInertiaMatrixAtCOG() const;


    private:

        friend std::ostream&operator<<(std::ostream& os, const FrInertiaTensor& inertia);
        std::ostream& cout(std::ostream& os) const;

    };


    namespace internal {

        /// Swap the frame convention (NWU/NED) of inertia coefficients
        inline void SwapInertiaFrameConvention(double& Ixx, double& Iyy, double& Izz,
                                               double& Ixy, double& Ixz, double& Iyz) {
            Ixy = -Ixy;
            Ixz = -Ixz;
        }

    }  // end namespace frydom::internal


}  // end namespace frydom

#endif //FRYDOM_FRINERTIATENSOR_H
