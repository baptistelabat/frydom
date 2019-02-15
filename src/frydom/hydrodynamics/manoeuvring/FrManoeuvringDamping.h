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

#include "frydom/core/force/FrForce.h"

namespace frydom {

    /**
     * \class FrManoeuvringDamping
     * \brief Class not used.
     */
    class FrManoeuvringDamping : public FrForce {


    };

    /**
     * \class FrTaylorManDamping
     * \brief Class not used.
     */
    class FrTaylorManDamping : public FrForce {

        struct TypeCoeff {
            double val = 0.;
            int cm = 0;
            int m1 = 0;
            int m2 = 0;
            int cn = 0;
            int n1 = 0;
            int n2 = 0;
            int cp = 0;
            int p1 = 0;
            int p2 = 0;
        };

    private:
        std::vector<TypeCoeff> m_cx;                    ///< Taylor coefficients for surge motion
        std::vector<TypeCoeff> m_cy;                    ///< Taylor coefficients for sway motion
        std::vector<TypeCoeff> m_cn;                    ///< Taylor coefficients for yaw motion

        TypeCoeff SetCoeff(const double val, const int m, const int n, const int p);
        TypeCoeff SetCoeff(const std::string tag, const double val);
        double ForceComponent(const TypeCoeff coeff, const double vx, const double vy, const double vrz) const;

    public:
        /// Default constructor
        FrTaylorManDamping() = default;

        /// Set taylor coefficients defined by tag
        void Set(const std::string tag, const double val);

        /// Set taylor coefficients for surge motion
        void SetX(const std::string tag, const double val);

        /// Set taylor coefficients for sway motion
        void SetY(const std::string tag, const double val);

        /// Set taylor coefficients for yaw motion
        void SetN(const std::string tag, const double val);

        void SetX(const double val, const int m, const int n, const int p);

        void SetY(const double val, const int m, const int n, const int p);

        void SetN(const double val, const int m, const int n, const int p);

        void UpdateState() override;

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "Fd_man_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }

    };


    /*
    class FrAddedMassManDamping : public FrForce {


    private:
        eigen::matrix<6, 6> A0;
    public:
        /// Default constructor
        FrManoeuvringDamping() {};

        //FrManoeuvringDamping(FrHydroDB* HDB, FrOffshoreSystem *system) {
        //
        //
        //}

        //void SetAddedMassCoeff();

        /// Update force
        void UpdateState() override {

            auto vel = GetBody()->GetCurrentRelativeVelocity();
            auto ang = GetBody()->GetRot_dt();

            auto u = std::vector<double>(vel.x(), vel.y(), vel.z(), ang.x(), ang.y(), ang.z());

            force = 0.;
            for (unsigned int i = 0; i < 6; i++) {
                force.x() += (A0.at(1, i) * u[5] - A0.at(2, i) * u[4]) * u[i];
                force.y() += (A0.at(2, i) * u[3] - A0.at(0, i) * u[5]) * u[i];
                force.z() += (A0.at(0, i) * u[4] - A0.at(1, i) * u[3]) * u[i];
            }

            moment = 0.;
            for (unsigned int i = 0; i < 6; i++) {
                moment.x() +=
                        (A0.at(1, i) * u[2] - A0.at(2, i) * u[1] + A0.at(4, i) * u[5] - A0.at(5, i) * u[4]) * u[i];
                moment.y() +=
                        (A0.at(2, i) * u[0] - A0.at(0, i) * u[2] + A0.at(5, i) * u[3] - A0.at(3, i) * u[5]) * u[i];
                moment.z() +=
                        (A0.at(0, i) * u[1] - A0.at(1, i) * u[0] + A0.at(3, i) * u[4] - A0.at(4, i) * u[3]) * u[i];
            }

        }

    };
     */

















    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

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
    class FrManDampingTaylorExpansion_ : public FrForce_ {

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
        FrManDampingTaylorExpansion_() = default;

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
        void StepFinalize() override {}

        /// Clear all coefficients definition
        void ClearAll();
    };





}

#endif //FRYDOM_FRMANOEUVRINGDAMPING_H
