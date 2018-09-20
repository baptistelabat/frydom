//
// Created by camille on 25/04/18.
//

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















    /// REFACTORING --------------->>>>>>>>>>>>>>>>




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

    class FrKinematicStretching_ {

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

    class FrKinStretchingExtrapol_ : public FrKinematicStretching_ {

    public:
        /// Return the vertical scaling coefficient with extrapolation stretching
        double Eval(const double& z, const double& konde, const double& depth) const override;

        /// Return the vertical derivative of the scale coefficient
        double EvalDZ(const double& z, const double& konde, const double& depth) const override;
    };

    // --------------------------------------------------------
    // Wheeler stretching
    // --------------------------------------------------------

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


    // -------------------------------------------------------------------
    // Chakrabarti
    // -------------------------------------------------------------------

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

    // ---------------------------------------------------------------------
    // Delta-stretching
    // ---------------------------------------------------------------------

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
        void SetParam(const double hd, const double delta);

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
