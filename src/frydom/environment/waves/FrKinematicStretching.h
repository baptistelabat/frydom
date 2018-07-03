//
// Created by camille on 25/04/18.
//

#ifndef FRYDOM_FRKINEMATICSTRETCHING_H
#define FRYDOM_FRKINEMATICSTRETCHING_H

#include "frydom/core/FrConstants.h"

namespace frydom {

    enum FrStretchingType {
        NO_STRETCHING,
        VERTICAL,
        EXTRAPOLATE,
        WHEELER,
        CHAKRABARTI,
        DELTA
    };

    // --------------------------------------------------------
    // Forward declaration
    // --------------------------------------------------------

    class FrWaveField;

    // --------------------------------------------------------
    // Base class for the kinematic stretching
    // --------------------------------------------------------

    class FrKinematicStretching {

    protected:
        bool is_steady = true;                      ///< The expression is not time dependant
        bool m_infinite_depth = false;              ///< Infinite depth is active

    public:
        /// Set the infinite depth value
        void SetInfDepth(const bool infinite_depth) { m_infinite_depth = infinite_depth; }

        /// Activate the infinite depth
        void SetInfDepth_ON() { this->SetInfDepth(true); }

        /// Set if the expresion is time dependant (is_steady=true) or not (is_steady=false)
        void SetSteady(const bool steady) { is_steady = steady; }

        /// Return true if the scale factor is not time dependant. False otherwise
        bool IsSteady() const { return is_steady; }

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const double& z, const double& konde, const double& depth) const {
            return Ez(z, konde, depth);
        }

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const double& x, const double& y, const double& z,
                                   const double& konde, const double& depth) const {
            return Ez(z, konde, depth);
        }

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const chrono::ChVector<>& pos, const double& konde, const double& depth) const {
            return Eval(pos.z(), konde, depth);
        }

        /// Return a list of vertical scale coefficient for linear wave velocity
        virtual std::vector<double> Eval(const double& x, const double& y, const double& z,
                            const std::vector<double>& vkonde, const double& depth) const {
            std::vector<double> result;
            for (auto& konde: vkonde) {
                result.push_back( Eval(x, y, z, konde, depth) );
            }
            return result;
        }

        /// Return the vertical derivative of the scale factor
        virtual double EvalDZ(const double& z, const double& konde, const double& depth) const {
            return diffEz(z, konde, depth);
        }

        /// Return the vertical derivative of the scale factor
        virtual double EvalDZ(const double&x, const double& y, const double& z,
                                     const double& konde, const double& depth) const {
            return EvalDZ(z, konde, depth);
        }

        /// Return the vertical derivatice of the scale factor
        virtual double EvalDZ(const chrono::ChVector<>& pos, const double& konde, const double& depth) const {
            return EvalDZ(pos.z(), konde, depth);
        }

        /// Return a list of vertical derivative of the scale factor
        virtual std::vector<double> EvalDZ(const double& x, const double& y, const double& z,
                                           const std::vector<double>& vkonde, const double& depth) const {
            std::vector<double> result;
            for (auto& konde: vkonde) {
                result.push_back( EvalDZ(x, y, z, konde, depth) );
            }
            return result;
        }

    protected:
        /// Definition of the vertical scale coefficient for linear wave velocity
        double Ez(const double& z, const double& konde, const double& depth) const {
            if (m_infinite_depth) {
                return exp(konde * z);
            } else {
                return cosh(konde*(z+depth)) / sinh(konde*depth);
            }
        }

        /// Definition of the derivative according to z-direction for the scale factor
        double diffEz(const double& z, const double& konde, const double& depth) const {
            if (m_infinite_depth) {
                return konde * exp(konde * z);
            } else {
                return konde* sinh(konde*(z+depth)) / sinh(konde*depth);
            }
        }
    };

    // ------------------------------------------------------
    // Vertical stretching
    // ------------------------------------------------------

    class FrKinStretchingVertical : public FrKinematicStretching {

    public:
        /// Return the vertical scaling coefficient with vertical stretching
        double Eval(const double& z, const double& konde, const double& depth) const override {
            if (z < DBL_EPSILON) {
                return Ez(z, konde, depth);
            } else {
                return Ez(0., konde, depth);
            }
        }

        /// Return the vertical derivative of the scale factor with vertical stretching
        double EvalDZ(const double& z, const double& konde, const double& depth) const override {
            if (z < DBL_EPSILON) {
                return diffEz(z, konde, depth);
            } else {
                return diffEz(0., konde, depth);
            }
        }
    };

    // -------------------------------------------------------
    // Extrapolation stretching
    // -------------------------------------------------------

    class FrKinStretchingExtrapol : public FrKinematicStretching {

    public:
        /// Return the vertical scaling coefficient with extrapolation stretching
        double Eval(const double& z, const double& konde, const double& depth) const override {
            if (z < DBL_EPSILON) {
                return Ez(z, konde, depth);
            } else
                return Ez(0., konde, depth) + konde * z;
        }

        /// Return the vertical derivative of the scale coefficient
        double EvalDZ(const double& z, const double& konde, const double& depth) const override {
            if (z < DBL_EPSILON) {
                return diffEz(z, konde, depth);
            } else {
                return diffEz(0., konde, depth) + konde;
            }
        }
    };

    // --------------------------------------------------------
    // Wheeler stretching
    // --------------------------------------------------------

    class FrKinStretchingWheeler : public FrKinematicStretching {

    private:
        FrWaveField* m_waveField;           ///< Wave field definition

    public:
        /// Default constructor
        explicit FrKinStretchingWheeler(FrWaveField* waveField) : m_waveField(waveField) {
            SetSteady(false);
        }

        /// Define the linear wave field to which the stretching is applied
        void SetWaveField(FrWaveField* waveField) { m_waveField = waveField; }

        double Eval(const double& z, const double& konde, const double& depth) const override {
            std::cout << "warning : 3D-coordinate is missing for wheeler stretching" << std::endl;
            return 1.;
        }

        double EvalDZ(const double& z, const double& konde, const double& depth) const override {
            std::cout << "warning : 3D-coordinate is missing for wheeler stretching" << std::endl;
            return 1.;
        }

        /// Return the vertical scaling coefficient with Wheeler stretching
        double Eval(const double& x, const double& y, const double& z,
                           const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale factor
        double EvalDZ(const double& x, const double& y, const double& z,
                             const double& konde, const double& depth) const override;

    };


    // -------------------------------------------------------------------
    // Chakrabarti
    // -------------------------------------------------------------------

    class FrKinStretchingChakrabarti : public FrKinematicStretching {

    private:
        FrWaveField* m_waveField;           ///< Wave field definition

    public:
        /// Default constructor
        explicit FrKinStretchingChakrabarti(FrWaveField* waveField) : m_waveField(waveField) {
                SetSteady(false);
        }

        /// Define the linear wave field to which the stretching is applied
        void SetWaveField(FrWaveField* waveField) { m_waveField = waveField; }

        /// Return the vertical scaling factor with the Chakrabati stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    };

    // ---------------------------------------------------------------------
    // Delta-stretching
    // ---------------------------------------------------------------------

    class FrKinStretchingDelta : public FrKinematicStretching {

    private:
        FrWaveField* m_waveField;           ///< Wave field definition
        double m_delta;                     ///< Delta parameter in [0 , 1]
        double m_hd;                        ///< water depth to which the delta-stretching is applied

    public:
        /// Default constructor
        explicit FrKinStretchingDelta(FrWaveField* waveField) : m_waveField(waveField),
                                                                m_delta(0.3), m_hd(0.)
        {
            SetSteady(false);
        }

        /// Define the linear wave field to which the stretching is applied
        void SetWaveField(FrWaveField* waveField) { m_waveField = waveField; }

        /// Define the water depth and delta parameters
        void SetParam(const double hd, const double delta) {
            m_hd = hd;
            m_delta = delta;
        }

        /// Return the vertical scaling factor with the delta stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    private:
        /// Return the modified vertical coordinate
        double Zp(const double& x, const double& y, const double& z) const;

    };

}


#endif //FRYDOM_FRKINEMATICSTRETCHING_H
