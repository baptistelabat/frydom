// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRKINEMATICSTRETCHING_H
#define FRYDOM_FRKINEMATICSTRETCHING_H

#include "chrono/core/ChVector.h"

namespace frydom {

    enum FrStretchingType {
        NO_STRETCHING,
        VERTICAL,
        EXTRAPOLATE,
        WHEELER,
        CHAKRABARTI,
        DELTA,
        HDELTA
    };

    // --------------------------------------------------------
    // Forward declaration
    // --------------------------------------------------------

    class FrWaveField;

    // --------------------------------------------------------
    // Base class for the kinematic stretching
    // --------------------------------------------------------

    /**
     * \class FrKinematicStretching
     * \brief Class for defining the kinematic stretching model.
     */
    class FrKinematicStretching {

    protected:
        bool is_steady = true;                      ///< The expression is not time dependant
        bool m_infinite_depth = false;              ///< Infinite depth is active

    public:
        /// Set the infinite depth value
        void SetInfDepth(const bool infinite_depth);

        /// Activate the infinite depth
        void SetInfDepth_ON();

        /// Set if the expresion is time dependant (is_steady=true) or not (is_steady=false)
        void SetSteady(const bool steady);

        /// Return true if the scale factor is not time dependant. False otherwise
        bool IsSteady() const;

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const double& z, const double& konde, const double& depth) const;

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const double& x, const double& y, const double& z,
                                   const double& konde, const double& depth) const;

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const chrono::ChVector<>& pos, const double& konde, const double& depth) const;

        /// Return a list of vertical scale coefficient for linear wave velocity
        virtual std::vector<double> Eval(const double& x, const double& y, const double& z,
                            const std::vector<double>& vkonde, const double& depth) const;

        /// Return the vertical derivative of the scale factor
        virtual double EvalDZ(const double& z, const double& konde, const double& depth) const;

        /// Return the vertical derivative of the scale factor
        virtual double EvalDZ(const double&x, const double& y, const double& z,
                                     const double& konde, const double& depth) const;

        /// Return the vertical derivatice of the scale factor
        virtual double EvalDZ(const chrono::ChVector<>& pos, const double& konde, const double& depth) const;

        /// Return a list of vertical derivative of the scale factor
        virtual std::vector<double> EvalDZ(const double& x, const double& y, const double& z,
                                           const std::vector<double>& vkonde, const double& depth) const;

    protected:
        /// Definition of the vertical scale coefficient for linear wave velocity
        double Ez(const double& z, const double& konde, const double& depth) const;

        /// Definition of the derivative according to z-direction for the scale factor
        double diffEz(const double& z, const double& konde, const double& depth) const;
    };

    // ------------------------------------------------------
    // Vertical stretching
    // ------------------------------------------------------

    /**
     * \class FrKinStretchingVertical
     * \brief Class for using the vertical stretching model.
     */
    class FrKinStretchingVertical : public FrKinematicStretching {

    public:
        /// Return the vertical scaling coefficient with vertical stretching
        double Eval(const double& z, const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale factor with vertical stretching
        double EvalDZ(const double& z, const double& konde, const double& depth) const override;
    };

    // -------------------------------------------------------
    // Extrapolation stretching
    // -------------------------------------------------------

    /**
     * \class FrKinStretchingExtrapol
     * \brief Class for using the extrapolation stretching model.
     */
    class FrKinStretchingExtrapol : public FrKinematicStretching {

    public:
        /// Return the vertical scaling coefficient with extrapolation stretching
        double Eval(const double& z, const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale coefficient
        double EvalDZ(const double& z, const double& konde, const double& depth) const override;
    };

    // --------------------------------------------------------
    // Wheeler stretching
    // --------------------------------------------------------

    /**
     * \class FrKinStretchingWheeler
     * \brief Class for using the Wheeler stretching model.
     */
    class FrKinStretchingWheeler : public FrKinematicStretching {

    private:
        FrWaveField* m_waveField;           ///< Wave field definition

    public:
        /// Default constructor
        explicit FrKinStretchingWheeler(FrWaveField* waveField);

        /// Define the linear wave field to which the stretching is applied
        void SetWaveField(FrWaveField* waveField);

        double Eval(const double& z, const double& konde, const double& depth) const override;

        double EvalDZ(const double& z, const double& konde, const double& depth) const override;

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

    /**
     * \class FrKinStretchingChakrabarti
     * \brief Class for using the Chakrabarti stretching model.
     */
    class FrKinStretchingChakrabarti : public FrKinematicStretching {

    private:
        FrWaveField* m_waveField;           ///< Wave field definition

    public:
        /// Default constructor
        explicit FrKinStretchingChakrabarti(FrWaveField* waveField);

        /// Define the linear wave field to which the stretching is applied
        void SetWaveField(FrWaveField* waveField);

        /// Return the vertical scaling factor with the Chakrabati stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    };

    // ---------------------------------------------------------------------
    // Delta-stretching
    // ---------------------------------------------------------------------

    /**
     * \class FrKinStretchingDelta
     * \brief Class for using the delta-stretching model.
     */
    class FrKinStretchingDelta : public FrKinematicStretching {

    private:
        FrWaveField* m_waveField;           ///< Wave field definition
        double m_delta;                     ///< Delta parameter in [0 , 1]
        double m_hd;                        ///< water depth to which the delta-stretching is applied

    public:
        /// Default constructor
        explicit FrKinStretchingDelta(FrWaveField* waveField);

        /// Define the linear wave field to which the stretching is applied
        void SetWaveField(FrWaveField* waveField);

        /// Define the water depth and delta parameters
        void SetParam(const double hd, const double delta);

        /// Return the vertical scaling factor with the delta stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    private:
        /// Return the modified vertical coordinate
        double Zp(const double& x, const double& y, const double& z) const;

    };















    // REFACTORING --------------->>>>>>>>>>>>>>>>




//    enum FrStretchingType {
//        NO_STRETCHING,
//        VERTICAL,
//        EXTRAPOLATE,
//        WHEELER,
//        CHAKRABARTI,
//        DELTA
//    };

    // --------------------------------------------------------
    // Forward declaration
    // --------------------------------------------------------

    class FrWaveField_;

    // --------------------------------------------------------
    // Base class for the kinematic stretching
    // --------------------------------------------------------

    /**
     * \class FrKinematicStretching_
     * \brief Class for defining the kinematic stretching model.
     */
    class FrKinematicStretching_ {

    protected:
        bool is_steady = true;                      ///< The expression is not time dependant
        bool c_infinite_depth = false;              ///< cache value of infinite_depth of FrWaveField

    public:
        /// Set the infinite depth value
        void SetInfDepth(bool infinite_depth);

        /// Activate the infinite depth
        void SetInfDepth_ON();

        /// Set if the expresion is time dependant (is_steady=true) or not (is_steady=false)
        void SetSteady(bool steady);

        /// Return true if the scale factor is not time dependant. False otherwise
        bool IsSteady() const;

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const double& z, const double& konde, const double& depth) const;

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const double& x, const double& y, const double& z,
                                   const double& konde, const double& depth) const;

        /// Return the vertical scale coefficient for linear wave velocity
        virtual double Eval(const chrono::ChVector<>& pos, const double& konde, const double& depth) const;

        /// Return a list of vertical scale coefficient for linear wave velocity
        virtual std::vector<double> Eval(const double& x, const double& y, const double& z,
                            const std::vector<double>& vkonde, const double& depth) const;

        /// Return the vertical derivative of the scale factor
        virtual double EvalDZ(const double& z, const double& konde, const double& depth) const;

        /// Return the vertical derivative of the scale factor
        virtual double EvalDZ(const double&x, const double& y, const double& z,
                                     const double& konde, const double& depth) const;

        /// Return the vertical derivatice of the scale factor
        virtual double EvalDZ(const chrono::ChVector<>& pos, const double& konde, const double& depth) const;

        /// Return a list of vertical derivative of the scale factor
        virtual std::vector<double> EvalDZ(const double& x, const double& y, const double& z,
                                           const std::vector<double>& vkonde, const double& depth) const;

    protected:
        /// Definition of the vertical scale coefficient for linear wave velocity
        double Ez(const double& z, const double& konde, const double& depth) const;

        /// Definition of the derivative according to z-direction for the scale factor
        double diffEz(const double& z, const double& konde, const double& depth) const;

    };

    // ------------------------------------------------------
    // Vertical stretching
    // ------------------------------------------------------

    /**
     * \class FrKinStretchingVertical_
     * \brief Class for using the vertical stretching model.
     */
    class FrKinStretchingVertical_ : public FrKinematicStretching_ {

    public:
        /// Return the vertical scaling coefficient with vertical stretching
        double Eval(const double& z, const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale factor with vertical stretching
        double EvalDZ(const double& z, const double& konde, const double& depth) const override;
    };

    // -------------------------------------------------------
    // Extrapolation stretching
    // -------------------------------------------------------

    /**
     * \class FrKinStretchingExtrapol_
     * \brief Class for using the extrapolation stretching model.
     */
    class FrKinStretchingExtrapol_ : public FrKinematicStretching_ {

    public:
        /// Return the vertical scaling coefficient with extrapolation stretching
        double Eval(const double& z, const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale coefficient
        double EvalDZ(const double& z, const double& konde, const double& depth) const override;
    };

    /// --------------------------------------------------------
    /// Wheeler stretching
    /// --------------------------------------------------------
    /// This method stretches the vertical scale, in order to get at z=eta, the kinematic given by the linear theory
    /// at z=0. This is done by replacing E(z) with E(z'), where z' = h (z-eta)/(h+eta),
    /// which means that z'+h = h (z+h)/(h+eta)
    /// h is the water depth and eta is the instantaneous wave elevation.
    /// The wheeler stretching is then defined for : h < z < eta

    /**
     * \class FrKinStretchingWheeler_
     * \brief Class for using the Wheeler stretching model.
     */
    class FrKinStretchingWheeler_ : public FrKinematicStretching_ {

    private:
        FrWaveField_* m_waveField;           ///< Wave field definition

    public:
        /// Default constructor
        explicit FrKinStretchingWheeler_(FrWaveField_* waveField);

//        /// Define the linear wave field to which the stretching is applied
//        void SetWaveField(FrWaveField* waveField);

        double Eval(const double& z, const double& konde, const double& depth) const override;

        double EvalDZ(const double& z, const double& konde, const double& depth) const override;

        /// Return the vertical scaling coefficient with Wheeler stretching
        double Eval(const double& x, const double& y, const double& z,
                           const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale factor
        double EvalDZ(const double& x, const double& y, const double& z,
                             const double& konde, const double& depth) const override;

    };


    /// -------------------------------------------------------------------
    /// Chakrabarti
    /// -------------------------------------------------------------------
    /// This method, proposed by Chakrabarti[1971], modifies the original expression ch(k(z+h))/sh(kh)
    /// by adapting the water depth. The previous term is then modified in           ch(k(z+h))/sh(k(h+eta))

    /**
     * \class FrKinStretchingChakrabarti_
     * \brief Class for using the Chakrabarti stretching model.
     */
    class FrKinStretchingChakrabarti_ : public FrKinematicStretching_ {

    private:
        FrWaveField_* m_waveField;           ///< Wave field definition

    public:
        /// Default constructor
        explicit FrKinStretchingChakrabarti_(FrWaveField_* waveField);

//        /// Define the linear wave field to which the stretching is applied
//        void SetWaveField(FrWaveField* waveField);

        /// Return the vertical scaling factor with the Chakrabati stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    };

    /// ---------------------------------------------------------------------
    /// Delta-stretching
    /// ---------------------------------------------------------------------
    /// This method is a coupling between the Wheeler and the extrapolation stretching, proposed by Rodenbush and Forristall[1986]
    /// It still replace E(z) by E(z'), but where
    ///  for z < -h_Delta : z' = z
    ///  for z > -h_Delta : z' = (z + h_Delta) (h_Delta + Delta.eta)/(h_Delta + eta) - h_Delta
    /// Delta is a parameter taken between 0 and 1, and h_Delta is the water height on which the stretching is applied
    /// if h_Delta = h and Delta = 0 --> Wheeler stretching
    /// if h_Delta = h and Delta = 1 --> extrapolation stretching
    /// Rodenbush and Forristall[1986] recommend using Delta = 0.3 and h_Delta = Hs/2. However Molin[2002] suggests that
    /// h_Delta = h, the water depth, is usually chosen.

    /**
     * \class FrKinStretchingDelta_
     * \brief Class for using the delta-stretching model.
     */
    class FrKinStretchingDelta_ : public FrKinematicStretching_ {

    private:
        FrWaveField_* m_waveField;           ///< Wave field definition
        double m_delta;                     ///< Delta parameter in [0 , 1]
        double m_hd;                        ///< water depth to which the delta-stretching is applied

    public:
        /// Default constructor
        explicit FrKinStretchingDelta_(FrWaveField_* waveField);

//        /// Define the linear wave field to which the stretching is applied
//        void SetWaveField(FrWaveField* waveField);

        /// Define the water depth and delta parameters
        void SetParam(double hd, double delta);

        /// Return the vertical scaling factor with the delta stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    private:
        /// Return the modified vertical coordinate
        double Zp(const double& x, const double& y, const double& z) const;

        /// Return the derivative modified vertical coordinate
        double DZp(const double& x, const double& y, const double& z) const;

    };

    /// ---------------------------------------------------------------------
    /// HDelta-stretching
    /// ---------------------------------------------------------------------
    /// Same method as Delta stretching but, h_Delta is fixed as h, the water depth

    /**
     * \class FrKinStretchingDelta_
     * \brief Class for using the Hdelta-stretching model.
     */
    class FrKinStretchingHDelta_ : public FrKinematicStretching_ {

    private:
        FrWaveField_* m_waveField;           ///< Wave field definition
        double m_delta = 0.3;                     ///< Delta parameter in [0 , 1]

    public:
        /// Default constructor
        explicit FrKinStretchingHDelta_(FrWaveField_* waveField);

//        /// Define the linear wave field to which the stretching is applied
//        void SetWaveField(FrWaveField* waveField);

        /// Define the water depth and delta parameters
        void SetDelta(double delta);

        /// Return the vertical scaling factor with the delta stretching
        double Eval(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

        /// Return the derivative of the vertical scaling factor
        double EvalDZ(const double& x, const double& y, const double& z, const double& konde, const double& depth) const override;

    private:
        /// Return the modified vertical coordinate
        double Zp(const double& x, const double& y, const double& z, const double& depth) const;

        /// Return the derivative modified vertical coordinate
        double DZp(const double& x, const double& y, const double& z, const double& depth) const;

    };

}


#endif //FRYDOM_FRKINEMATICSTRETCHING_H
