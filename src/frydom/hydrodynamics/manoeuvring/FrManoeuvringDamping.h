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


#ifndef FRYDOM_FRMANOEUVRINGDAMPING_H
#define FRYDOM_FRMANOEUVRINGDAMPING_H


#include <utility>

#include "frydom/core/force/FrForce.h"



namespace frydom {

    /// This class defined a generic manoeuvring damping model in surge, sway and yaw. Each component of the damping force
    /// is defined as the sum of non-linear terms depending in surge, sway and yaw velocity of the body.
    /// Specific manoeuvring damping term can be defined with shape definition (ex: "Xuuv") with :
    /// - the first capital letter corresponding the component axis :
    ///     X: component in surge
    ///     Y: component in sway
    ///     N: component in yaw
    /// - the lower case letters corresponding to the exponent applied on ship velocity :
    ///     u: translational velocity in surge
    ///     v: translational velocity in sway
    ///     w: angular velocity in yaw
    class FrManDampingTaylorExpansion : public FrForce {

        /// This structure contains the parameters defining the terms in manoeuvring damping force
        struct TypeParam_ {
            double val = 0.;
            bool cm = false;
            bool cn = false;
            bool cp = false;
            std::pair<int, int> m;
            std::pair<int, int> n;
            std::pair<int, int> p;
        };

    private:
        std::vector<TypeParam_> m_cx;   ///< Parameters in surge motion
        std::vector<TypeParam_> m_cy;   ///< Parameters in sway motion
        std::vector<TypeParam_> m_cn;   ///< Parameters in yaw motion

        TypeParam_ SetParams(double val, int m, int n, int p);

        TypeParam_ SetParams(std::string tag, double val);

        double ForceComponent(const TypeParam_ coeff, double vx, double vy, double vrz) const;

    public:

        /// Default constructor
        FrManDampingTaylorExpansion() = default;

        /// Definition of a manoeuvring force component from string definition and scalar value
        /// \param tag : shape definition (ex: "Xuuv")
        /// \param val : coefficient value
        void Set(std::string tag, double val);

        /// Definition of a manoeuvring force component in surge from string definition and scalar value
        /// \param tag : shape definition (ex: "uuv")
        /// \param val : coefficient value
        void SetX(std::string tag, double val);

        /// Definition of a manoeuvring force component in sway from string definition and scalar value
        /// \param tag : shape definition (ex: "uuv")
        /// \param val : coefficient value
        void SetY(std::string tag, double val);

        /// Definition of a manoeuvring force component in yaw from string definition and scalar value
        /// \param tag : shape definition (ex: "uuv")
        /// \param val : coefficient value
        void SetN(std::string tag, double val);

        /// Definition of a manoeuvring force component in surge from exponent definition
        /// \param val : coefficient value
        /// \param m : exponent for surge velocity
        /// \param n : exponent for sway velocity
        /// \param p : exponent for yaw velocity
        void SetX(double val, int m, int n, int p);

        /// Definition of a manoeuvring force component in sway from exponent definition
        /// \param val : coefficient value
        /// \param m : exponent for surge velocity
        /// \param n : exponent for sway velocity
        /// \param p : exponent for yaw velocity
        void SetY(double val, int m, int n, int p);

        /// Definition of a manoeuvring force component in yaw from exponent
        /// \param val : coefficient value
        /// \param m : exponent for surge velocity
        /// \param n : exponent for sway velocity
        /// \param p : exponent for yaw velocity
        void SetN(double val, int m, int n, int p);

        /// Update of the manoeuvring force value
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Initialization of the manoeuvring force model
        void Initialize() override;

        /// Applied method at the end of each time step
        void StepFinalize() override;

        /// Clear all coefficients definition
        void ClearAll();
    };

}  // end namespace frydom

#endif //FRYDOM_FRMANOEUVRINGDAMPING_H
