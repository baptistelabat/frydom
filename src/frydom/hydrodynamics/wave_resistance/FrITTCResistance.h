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


#ifndef FRYDOM_FRITTC57_H
#define FRYDOM_FRITTC57_H

#include "frydom/core/force/FrForce.h"


namespace frydom {

    // Forward Declaration
    class FrEnvironment_;

    /**
     * \class FrITTCResistance_
     * \brief Class for computing the wave resistance.
     */
    class FrITTCResistance_ : public FrForce_ {

      private:

        double m_Lpp;  ///< Characteristic length (length between perpendicular for ships) (meters)
        double m_hullWetSurface;    ///< Hull wetted surface (m**2)
        double m_k;    ///< Hull form factor
        double m_cr;   ///< residuary coefficient
        double m_ca;   ///< surface roughness coefficient
        double m_caa;  ///< air resistance coefficient
        double m_capp; ///< appendage resistance coefficient

        FrEnvironment_* m_environment;

      public:

        /// Constructor of a new resistance force from ITTC standard model
        /// \param Lpp Length between perpendicular
        /// \param hullWetSurface Wetted surface area
        /// \param cr Resistance coefficient
        /// \param k Hull form coefficient
        /// \param ca Surface roughness coefficient
        /// \param caa Air resistance coefficient
        /// \param capp Appendage resistance coefficient
        FrITTCResistance_(double Lpp, double hullWetSurface, double cr, double k = 0.,
                         double ca = 0., double caa=0., double capp=0.) :
            m_Lpp(Lpp), m_hullWetSurface(hullWetSurface), m_k(k), m_cr(cr), m_ca(ca), m_caa(caa), m_capp(capp) {}

        /// Set the length between perpendicular of the ship
        /// \param Lpp Length between perpendicular (m)
        void SetLpp(double Lpp) { m_Lpp = Lpp; }

        /// Set the hull form factor
        /// \param k hull form factor
        void SetHullFormFactor(double k) { m_k = k; }

        /// Set the residual coefficient
        /// Cr = Ctm - (1+k) x Cfm
        /// where Ctm is the total measured resistance coefficient
        ///       Cfm is the calculated model frictional coefficient
        ///       k is the hull form factor
        /// \param cr residual coefficient
        void SetResidualCoefficient(double cr) { m_cr = cr; }

        /// Set the roughness allowance coefficient
        /// \param ca roughness allowance coefficient
        void SetRoughnessCoefficient(double ca) { m_ca = ca; }

        /// Set the roughness allowance coefficient from length at the waterline and surface roughness
        /// ca = (105 x ( ks / Lwl)^1/3 - 0.64) x 10^-3 (from ITTC78)
        /// where ks is the surface roughness (150x10^-6 by default)
        ///       Lwl the length at the waterline
        /// \param Lwl Characteristic length at the waterline, in meters
        /// \param surfaceRoughness Surface roughness, in meters
        void SetRoughnessFromLength(double Lwl, double surfaceRoughness = 1.5e-4);

        /// Set the air resistance coefficient.
        /// caa = frontal area / (1000. x wetted surface) [in ITTC78]
        /// \param caa air resistance coefficient
        void SetAirResistanceCoefficient(double caa) { m_caa = caa; }

        /// Set the air resistance coefficient from frontal area above the water level
        /// \param area Projected frontal area of the ship part above the water level
        void SetAirResistanceFromArea(double area);

        /// Set the appendage resistance coefficient
        /// \param capp appendage resistance coefficient
        void SetAppendageCoefficient(double capp) { m_capp = capp; }

        /// Update the force
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Initialization of the force model
        void Initialize() override;

        /// Methods to be applied at the end of each time step
        void StepFinalize() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRITTC57_H
