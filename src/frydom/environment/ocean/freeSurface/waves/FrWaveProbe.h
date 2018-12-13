//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRWAVEPROBE_H
#define FRYDOM_FRWAVEPROBE_H


#include "frydom/core/FrObject.h"





//#include <memory>
//#include <complex>
//
//#include "chrono/core/ChVector.h"
//

//#include "FrWaveField.h"
//#include "frydom/core/FrNode.h"
#include "chrono/core/ChFrameMoving.h"
//#include "frydom/core/FrEulerAngles.h"

//namespace chrono {
//    template <class real>
//    class ChFrameMoving;
//}



namespace frydom {

    // =================================================================================================================
    // Forward declaration
    class FrWaveField;
    class FrRamp;

    /// Class to make a measurement of the wave elevation with respect to the mean water height (tidal)
    /// It is mainly used at a fixed absolute position in the local horizontal plane
    class FrWaveProbe : public FrObject {

    protected:
//        FrWaveField* m_waveField = nullptr;                           ///< Wave field

        double m_x = 0;                                     ///< Abs position in x of the sensor (m)
        double m_y = 0;                                     ///< Abs position in y of the sensor (m)

        std::shared_ptr<chrono::ChFrameMoving<double>> m_node;      ///< Node to which the wave probe is attached
        std::shared_ptr<FrRamp> m_waveRamp;                 ///< Ramp applied at the initial stage

    public:

        FrWaveProbe() = default;

        /// New constructor from sensor position in the horizontal plane
        FrWaveProbe(double x, double y);

        /// Set Abs position in X
        void SetX(double x);

        /// Return the abs position in X of the sensor
        double& GetX() const;

        /// Set Abs position in Y
        void SetY(double y);

        /// Return the abs position in Y of the sensor
        double& GetY() const;

        std::shared_ptr<chrono::ChFrameMoving<>> GetNode() const;

//        /// Return the wave field
//        virtual FrWaveField* GetWaveField() const = 0;

        /// Initialization of the wave probe sensor
        void Initialize() override {};

        /// Method run after each time step
        void StepFinalize() override {}

        /// Attached a node to the wave probe
        void AttachedNode(std::shared_ptr<chrono::ChFrameMoving<double>> node);


        /// Return the free surface elevation at the sensor position
        virtual double GetElevation(double time) const = 0;

        /// Return the wave frequencies ate the probe location
        virtual std::vector<double> GetFrequencies() const = 0;

    };

    // =================================================================================================================
    // Forward declaration
    class FrLinearWaveField;

    /// Specialization of the wave probe for linear wave field and moving frame.
    /// Assumed moving frame and encounter frequency

    class FrLinearWaveProbe : public FrWaveProbe {

    protected:
        FrLinearWaveField* m_waveField;

    public:

        explicit FrLinearWaveProbe(FrLinearWaveField *waveField);

        FrLinearWaveProbe(FrLinearWaveField *waveField, double x, double y);

//        /// Set the linear wave field linked to the wave probe
//        void SetWaveField(FrLinearWaveField *waveField);

        /// Return the linear wave field to which the wave probe is linked
        FrLinearWaveField* GetWaveField() const;

        /// Compute complex elevation depending on the position and frequency
        std::vector<std::vector<std::complex<double>>> GetCmplxElevation() const;

        /// Return the wave elevation at the wave probe position
        double GetElevation(double time) const override;

        /// Return the wave frequencies at the wave probe position
        std::vector<double> GetFrequencies() const override;

        /// Return the encounter wave frequencies at the wave probe position
        std::vector<std::vector<double>> GetEncounterWaveFrequencies() const;

    };

    /// Specialization of the wave probe to optimize cpu time for linear wave field
    /// Assumed linear wave field and fixe position if the global reference frame.
    /// Assumed only time frequency dependance.

    class FrLinearWaveProbeSteady : public FrLinearWaveProbe {  // TODO: mettre en cache la steady elevation pour les params x, y

    private:
        std::vector<std::complex<double>> m_steadyElevation;            ///< steady part of the wave elevation

    public:
        explicit FrLinearWaveProbeSteady(FrLinearWaveField* waveField);

        /// Constructor of the wave sensor with position in the horizontal plane
        FrLinearWaveProbeSteady(FrLinearWaveField* waveField, double x, double y);

        /// Set the steady part of the wave elevation
        void Initialize() override;

        /// Return the wave elevation at the sensor position
        double GetElevation(double time) const override;

        /// Return the wave frequencies
        std::vector<double> GetFrequencies() const override;

    };










    // REFACTORING -------------->>>>>>>>>>>>

//
//
//
//
//
//    // =================================================================================================================
//    // Forward declaration
//    class FrWaveField_;
//    class FrRamp;
//
//    /// Class to make a measurement of the wave elevation with respect to the mean water height (tidal)
//    /// It is mainly used at a fixed absolute position in the local horizontal plane
//    class FrWaveProbe_ : public FrObject {
//
//    protected:
//        double m_x = 0;                                     ///< Abs position in x of the sensor (m)
//        double m_y = 0;                                     ///< Abs position in y of the sensor (m)
//        FrWaveField_* m_waveField;                           ///< Wave field
//        std::shared_ptr<chrono::ChFrameMoving<double>> m_node;      ///< Node to which the wave probe is attached
//        std::shared_ptr<FrRamp> m_waveRamp;                 ///< Ramp applied at the initial stage
//
//    public:
//        /// Default constructor
//        explicit FrWaveProbe_(FrWaveField_* waveField);
//
//        /// New constructor from sensor position in the horizontal plane
//        FrWaveProbe_(FrWaveField_* waveField, double x, double y);
//
//        /// Set Abs position in X
//        void SetX(double x);
//
//        /// Return the abs position in X of the sensor
//        double& GetX() const;
//
//        /// Set Abs position in Y
//        void SetY(double y);
//
//        /// Return the abs position in Y of the sensor
//        double& GetY() const;
//
//        std::shared_ptr<chrono::ChFrameMoving<>> GetNode() const;
//
//        /// Return the wave field
////        virtual FrWaveField* GetWaveField() const = 0;
//
//        /// Initialization of the wave probe sensor
//        virtual void Initialize() override {};
//
//        /// Method run after each time step
//        virtual void StepFinalize() override {}
//
//        /// Attached a node to the wave probe
//        void AttachedNode(std::shared_ptr<chrono::ChFrameMoving<double>> node);
//
//
//        /// Return the free surface elevation at the sensor position
//        virtual double GetElevation(double time) const = 0;
//
//        /// Return the wave frequencies ate the probe location
//        virtual std::vector<double> GetFrequencies() const = 0;
//
//    };
//
//    // =================================================================================================================
//    // Forward declaration
//    class FrLinearWaveField_;
//
//    /// Specialization of the wave probe for linear wave field and moving frame.
//    /// Assumed moving frame and encounter frequency
//
//    class FrLinearWaveProbe_ : public FrWaveProbe_ {
//
//    public:
//        explicit FrLinearWaveProbe_(FrLinearWaveField_* waveField);
//
//        FrLinearWaveProbe_(FrLinearWaveField_* waveField, double x, double y);
//
//        /// Set the linear wave field linked to the wave probe
////        void SetWaveField(FrLinearWaveField_ *waveField);
//
//        /// Return the linear wave field to which the wave probe is linked
////        FrLinearWaveField_* GetWaveField() const override;
//
//        /// Compute complex elevation depending on the position and frequency
//        std::vector<std::vector<std::complex<double>>> GetCmplxElevation() const;
//
//        /// Return the wave elevation at the wave probe position
//        double GetElevation(double time) const override;
//
//        /// Return the wave frequencies at the wave probe position
//        std::vector<double> GetFrequencies() const override;
//
//        /// Return the encounter wave frequencies at the wave probe position
//        std::vector<std::vector<double>> GetEncounterWaveFrequencies() const;
//
//    };
//
//    /// Specialization of the wave probe to optimize cpu time for linear wave field
//    /// Assumed linear wave field and fixe position if the global reference frame.
//    /// Assumed only time frequency dependance.
//
//    class FrLinearWaveProbeSteady_ : public FrLinearWaveProbe_ {  // TODO: mettre en cache la steady elevation pour les params x, y
//
//    private:
//        std::vector<std::complex<double>> m_steadyElevation;            ///< steady part of the wave elevation
//
//    public:
//        explicit FrLinearWaveProbeSteady_(FrLinearWaveField_* waveField): FrLinearWaveProbe_(waveField) {}
//
//        /// Constructor of the wave sensor with position in the horizontal plane
//        FrLinearWaveProbeSteady_(FrLinearWaveField_* waveField, double x, double y) : FrLinearWaveProbe_(waveField, x, y) {}
//
//        /// Set the steady part of the wave elevation
//        void Initialize() override;
//
//        /// Return the wave elevation at the sensor position
//        double GetElevation(double time) const override;
//
//        /// Return the wave frequencies
//        std::vector<double> GetFrequencies() const override;
//
//    };

}  // end namespace frydom


#endif //FRYDOM_FRWAVEPROBE_H

