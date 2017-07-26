//
// Created by frongere on 13/07/17.
//

#ifndef FRYDOM_FRCURRENTPOLARCOEFFS_H
#define FRYDOM_FRCURRENTPOLARCOEFFS_H


#include <frydom/core/FrConstants.h>
#include "frydom/misc/FrLookupTable1D.h"


namespace frydom {
namespace environment {

    class FrCurrentPolarCoeffs : public FrLookupTable1D<double> {

    private:

        FrFrame  m_frame     = NED;
        bool     m_symmetric = true;
        bool     m_negate    = false;

    public:


        void Initialize(const std::vector<double>& angles,
                        const std::vector<double>& cx,
                        const std::vector<double>& cy,
                        const std::vector<double>& cz);

        void Initialize(std::string yaml_file);

        void SetFrame(FrFrame frame) { m_frame = frame; }
        FrFrame GetFrame() const { return m_frame; }

        void SetSymmetric(const bool symmetry) { m_symmetric = symmetry; }
        bool IsSymmetric() const { return m_symmetric; }

        void SetNegate(const bool negate) { m_negate = negate; }
        bool GetNegate() const { return m_negate; }

        // ================================================
        // Coefficients evaluations
        // ================================================

        double CX(const double angle, FrFrame frame = NWU) const {
            // CX coefficient is pair so CX(alpha) = CX(-alpha) + no difference between NED and NWU frames
            double alpha = angle;
            auto cx = Eval("cx", fabs(alpha));
            if (m_negate) cx = -cx;
            return cx;
        }

        double CY(const double angle, FrFrame frame = NWU) const {
            return CYCZ(angle, "cy", frame);
        }

        double CZ(const double angle, FrFrame frame = NWU) const {
            return CYCZ(angle, "cz", frame);
        }

    private:
        inline double CYCZ(const double angle, const std::string& coeff, const FrFrame frame) const {

            double alpha = angle;

            // Making alpha following the same frame as data
            if (frame != m_frame) alpha = -alpha;

            // Getting the value in the table
            double cc;
            if (m_symmetric) {
                cc = Eval(coeff, fabs(alpha));
            } else {
                cc = Eval(coeff, alpha);
            }

            // Using the sign of the angle
            if (alpha < 0. && m_symmetric) cc = -cc;

            // Back to the requested frame
            if (frame != m_frame) cc = -cc;

            if (m_negate) cc = -cc;

            return cc;
        }
    };

}  // end namespace environment
}  // end namespace frydom


#endif //FRYDOM_FRCURRENTPOLARCOEFFS_H
