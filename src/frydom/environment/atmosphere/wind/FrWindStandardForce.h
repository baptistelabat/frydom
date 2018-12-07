//
// Created by camille on 17/07/18.
//

#ifndef FRYDOM_FRWINDSTANDARDFORCE_H
#define FRYDOM_FRWINDSTANDARDFORCE_H

#include "frydom/core/FrForce.h"

namespace frydom {

    class FrHydroBody;

    /// Standard wind drag force from DNV standard
    /// DNV-GL station keeping 01111

    class FrWindStandardForce : public FrForce {

    private:
        double m_rho_air = 1.226;                   ///< air density [kg.m^-3]
        double m_transverse_area;                   ///< frontal projected wind area [m^2]
        double m_lateral_area;                      ///< longitudinal projected wind area [m^2]
        double m_xc;                                ///< longitudinal position of the lateral area center (local frame)
        double m_lpp;                               ///< length between perpendicular [m]

    public:
        /// Default constructor
        FrWindStandardForce() = default;

        /// Constructor from body parameters
        FrWindStandardForce(const std::shared_ptr<FrHydroBody> mybody);

        /// Define the air density parameter used in drag force computation (kg/m^3)
        void SetAirDensity(const double rho_air) { m_rho_air = rho_air; }

        /// Return the air density parameter (kg/m^3)
        double GetAirDensity() const { return m_rho_air; }

        /// Define the maximum breadth at the waterline (m)
        void SetLateralArea(const double lateral_area) { m_lateral_area = lateral_area; }

        /// Return the lateral surface area (m^2)
        double GetLateralArea() const { return m_lateral_area; }

        /// Define the transverse frontal area (m^2)
        void SetTransverseArea(const double transverse_area) { m_transverse_area = transverse_area; }

        /// Return the transverse frontal area (m^2)
        double GetTransverseArea() const { return m_transverse_area; }

        /// Define the lateral position of the center area (in local frame)
        void SetXc(const double xc) { m_xc = xc; }

        /// Return the lateral position of the center area (in local frame)
        double GetXc() const { return m_xc; }

        /// Define the length between perpendicular (m)
        void SetLpp(const double Lpp) { m_lpp = Lpp; }

        /// Return the length between perpendicular (m)
        double GetLpp() const { return m_lpp; }

        /// Update value of the drag force
        void UpdateState() override;

    };















    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    /// Standard wind drag force from DNV standard
    /// DNV-GL station keeping 01111

    class FrWindStandardForce_ : public FrForce_ {

    private:
        double m_transverseArea = -1;
        double m_lateralArea = -1;
        double m_lpp = -1;
        double m_xCenter;

    public:

        FrWindStandardForce_() = default;

        void SetLateralArea(double lateralArea);

        double GetLateralArea() const { return m_lateralArea; }

        void SetTransverseArea(double transverseArea);

        double GetTransverseArea() const { return m_transverseArea; }

        void SetXCenter(double xCenter);

        double GetXCenter() const { return m_xCenter; }

        void SetLenghtBetweenPerpendicular(double lpp);

        double GetLengthBetweenPerpendicular() const { return m_lpp; }

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;
    };


}


#endif //FRYDOM_FRWINDSTANDARDFORCE_H
