//
// Created by camille on 17/07/18.
//

#ifndef FRYDOM_FRCURRENTSTANDARDFORCE_H
#define FRYDOM_FRCURRENTSTANDARDFORCE_H

#include "frydom/core/FrForce.h"

namespace frydom {

    /// Standard current drag force from DNV standard
    /// DNV-GL Station Keeping 01111

    class FrCurrentStandardForce : public FrForce {

    private:
        double m_rho;                       ///< water density [kg.m^-3]
        double m_breadth;                   ///< maximum breadth at the waterline [m]
        double m_draft;                     ///< summer load line draft [m]
        double m_lateral_area;              ///< longitudinal projected submerged area [m^2]
        double m_transverse_area;           ///< transverse lateral submerged area
        double m_xc;                        ///< longitudinal position of the area center of lateral area (body frame)
        double m_Lpp;                       ///< length between perpendicular (m)

    public:
        /// Default constructor
        FrCurrentStandardForce() = default;

        /// Constructor of the force from body parameters
        FrCurrentStandardForce(const std::shared_ptr<FrHydroBody> mybody);

        /// Define the water density parameter used in drag force computation (kg/m^3)
        void SetWaterDensity(const double rho) { m_rho = rho; }

        /// Return the water density parameter (kg/m^3)
        double GetWaterDensity() const { return m_rho; }

        /// Define the maximum breadth at the waterline (m)
        void SetMaxBreadth(const double breadth) { m_breadth = breadth; }

        /// Return the maximum breadth at the waterline (m)
        double GetMaxBreadth() const { return m_breadth; }

        /// Define the summer load draft line (m)
        void SetDraft(const double draft) { m_draft = draft; }

        /// Return the summer load draft line (m)
        double GetDraft() const { return m_draft; }

        /// Define the lateral surface area (m^2)
        void SetLateralArea(const double lateral_area) { m_lateral_area = lateral_area; }

        /// Return the lateral surface area (m^2)
        double GetLateralArea() const { return m_lateral_area; }

        /// Define the transverse underwater area (m^2)
        void SetTransverseArea(const double transverse_area) { m_transverse_area = transverse_area; }

        /// Return the transverse underwater area (m^2)
        double GetTransverseArea() const { return m_transverse_area; }

        /// Define the lateral position of the center area (local reference frame)
        void SetXc(const double xc) { m_xc = xc; }

        /// Return the lateral position of the center area (local reference frame)
        double GetXc() const { return m_xc; }

        /// Define the length between perpendicular (m)
        void SetLpp(const double Lpp) { m_Lpp = Lpp; }

        /// Return the length between perpendicular (m)
        double GetLpp() const { return m_Lpp; }

        /// Update value of the drag force
        void UpdateState() override;


    };

}



#endif //FRYDOM_FRCURRENTSTANDARDFORCE_H
