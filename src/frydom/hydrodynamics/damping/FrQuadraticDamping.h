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


#ifndef FRYDOM_FRQUADRATICDAMPING_H
#define FRYDOM_FRQUADRATICDAMPING_H

//#include "frydom/core/force/FrForce.h"
//#include "frydom/environment/FrFluidType.h"

namespace frydom {

     /**
     * \class FrQuadraticDamping_
     * \brief Class implementing a quadratic damping force:
      * Fqd = -1/2*rho*S*C*v*|v|
      * the velocity of the body,v, must be expressed in the local (body) frame,
      * but can be taken relatively or not to the current velocity (using m_relativeVelocity).
      * The resulting damping force (calculated in local frame) is then transformed in global frame.
     * */
    class FrQuadraticDamping_ : public FrForce_ {

    private:
        /// Damping coefficients in translation.
        double m_Cu = 0;
        double m_Cv = 0;
        double m_Cw = 0;

        /// Projected sections along each directions.
        double m_Su = 0;
        double m_Sv = 0;
        double m_Sw = 0;

        /// Specify if the body velocity is taken relatively to the current or not.
        FLUID_TYPE m_fluidType;
        bool m_relative2Fluid = false;

    public:

        FrQuadraticDamping_(FLUID_TYPE ft, bool relativeToFluid);

        /// Setter for the damping coefficients.
        void SetDampingCoefficients(double Cu, double Cv, double Cw);

        /// Getter for the damping coefficients.
        void GetDampingCoefficients(double& Cu, double& Cv, double& Cw);

        /// Setter for the projected sections.
        void SetProjectedSections(double Su, double Sv,double Sw);

        /// Getter for the projected sections.
        void GetProjectedSections(double& Su, double& Sv,double& Sw);

        /// Setter for the boolean : m_relativeVelocity
        void SetRelative2Fluid(bool relativeVelocity);

        /// Getter for the boolean : m_relativeVelocity
        bool GetRelative2Fluid();

        /// Initialize method checking if projected sections are correctly given, and initializing the logs.
        void Initialize() override;

        void StepFinalize() override;

        void Update(double time) override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRQUADRATICDAMPING_H
