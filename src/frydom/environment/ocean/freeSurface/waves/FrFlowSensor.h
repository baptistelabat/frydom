//
// Created by camille on 18/04/18.
//

#ifndef FRYDOM_FRFLOWSENSOR_H
#define FRYDOM_FRFLOWSENSOR_H


#include "frydom/core/FrObject.h"

#include "chrono/core/ChVector.h"



//#include "frydom/core/FrConstants.h"
//
#include "FrWaveField.h"
//#include "frydom/utils/FrUtils.h"




namespace frydom {


    // Forward declarations
    class FrWaveField;
    class FrRamp;


    /// Class to measure the velocity and pressure of a flow at a specific point
    /// Useful when the position of the measurement is fixed in the global coordinate system
    /// to limit computation time.

    class FrFlowSensor : public FrObject {

    protected:

        FrWaveField* m_waveField;       ///< Wave field defined in the system environment

        double m_x = 0;         ///< Position of the sensor in global coordinate system
        double m_y = 0;         // FIXME : utiliser ChVector plutôt ?
        double m_z = 0;

        std::shared_ptr<FrRamp> m_waveRamp;         // FIXME : pourquoi la rampe est dans le capteur ?

    public:
//        /// Default constructor
//        explicit FrFlowSensor(FrWaveField *waveField);
//
//        /// Constructor with position only
//        FrFlowSensor(FrWaveField *waveField, double x, double y, double z);;

        /// Constructor with wave field
        explicit FrFlowSensor(FrWaveField* waveField);;

        /// Constructor operator with 3D-coordinates
        FrFlowSensor(FrWaveField* waveField, double x, double y, double z);

        /// Constructor operator with position vector (3D)
//        FrFlowSensor(chrono::ChVector<> pos);

//        FrFlowSensor(FrWaveField* waveField, chrono::ChVector<> pos);


//        /// Define the x-coordinate in global coordinate system
//        void SetX(double x);
//        double GetX() const;
//
//        /// Define the y-coordinate in global coordinate system
//        void SetY(double y);
//        double GetY() const;
//
//        /// Define the z-coordinate in global coordinate system
//        void SetZ(double z);
//        double GetZ() const;

        /// Define the vector position of the sensor in global coordinate system
//        void SetPos(const chrono::ChVector<> pos);
        void SetPos(double x, double y, double z);
        void GetPos(double& x, double& y, double& z);
//        chrono::ChVector<>* GetPos() const;

        /// Return the wave field pointer (implemented in child class)
        virtual FrWaveField* GetWaveField() const = 0;

        /// Return the flow velocity at the location of the sensor
        virtual chrono::ChVector<double> GetVelocity() const;

        /// Return the flow velocity at the location of the sensor at time t
        virtual chrono::ChVector<double> GetVelocity(double time) const;

        /// Return the flow acceleration at the location of the sensor
        virtual chrono::ChVector<double> GetAcceleration() const;

        /// Return the flow acceleration at the location of the sensor at time t
        virtual chrono::ChVector<double> GetAcceleration(double time) const;

        void Initialize() override {};

        void StepFinalize() override {};

    };

    // Forwward declaration
    class FrLinearWaveField;


    /// Class to measure the flow velocity and pressure at a specific point.
    /// Used when linear wave field is activated

    class FrLinearFlowSensor : public FrFlowSensor {

    private:
        FrLinearWaveField* m_waveField = nullptr;           ///< Linear wave field pointer
        std::vector<chrono::ChVector<std::complex<double>>> m_steadyVelocity; ///< Steady state for effective time computation

    public:

        /// Constructor with wave field
        explicit FrLinearFlowSensor(FrLinearWaveField* waveField);

        /// Constructor with 3D-coordinates
        FrLinearFlowSensor(FrWaveField* waveField, double x, double y, double z);

        ~FrLinearFlowSensor();

//        FrLinearFlowSensor(FrWaveField* waveField, chrono::ChVector<> pos);

//        void SetWaveField(FrLinearWaveField* waveField);

        FrLinearWaveField* GetWaveField() const override;

        void Initialize() override;

        chrono::ChVector<double> GetVelocity(double time) const override;

        /// Return the flow acceleration at the location of the sensor at time t
        chrono::ChVector<double> GetAcceleration(double time) const override;

        /// Return the flow velocity at the location of the sensor
        chrono::ChVector<double> GetVelocity() const override;

        /// return the flow acceleration at the location of the sensor
        chrono::ChVector<double> GetAcceleration() const override;

    };















    /// REFACTORING ---------------->>>>>>>>>>>>>>>>

//
//
//
//
//
//    // Forward declarations
//    class FrWaveField_;
//    class FrRamp;
//
//
//    /// Class to measure the velocity and pressure of a flow at a specific point
//    /// Useful when the position of the measurement is fixed in the global coordinate system
//    /// to limit computation time.
//
//    class FrFlowSensor_ : public FrObject {
//
//    protected:
//
//        FrWaveField_* m_waveField;       ///< Wave field defined in the system environment
//
//        double m_x = 0;         ///< Position of the sensor in global coordinate system
//        double m_y = 0;         // FIXME : utiliser ChVector plutôt ?
//        double m_z = 0;
//
//        std::shared_ptr<FrRamp> m_waveRamp;         // FIXME : pourquoi la rampe est dans le capteur ?
//
//    public:
//
//        /// Constructor with wave field
//        explicit FrFlowSensor_(FrWaveField_* waveField);
//
//        /// Constructor operator with 3D-coordinates
//        FrFlowSensor_(FrWaveField_* waveField, double x, double y, double z);
//
////        /// Define the x-coordinate in global coordinate system
////        void SetX(const double x);
////        double GetX() const;
////
////        /// Define the y-coordinate in global coordinate system
////        void SetY(const double y);
////        double GetY() const;
////
////        /// Define the z-coordinate in global coordinate system
////        void SetZ(const double z);
////        double GetZ() const;
//
//        /// Define the vector position of the sensor in global coordinate system
//        void SetPos(double x, double y, double z);
////        chrono::ChVector<>* GetPos() const;
//        void GetPos(double& x, double& y, double& z) const;
//
//        /// Return the wave field pointer (implemented in child class)
//        virtual FrWaveField_* GetWaveField() const;
//
//        /// Return the flow velocity at the location of the sensor
//        virtual Velocity GetVelocity() const;
//
//        /// Return the flow velocity at the location of the sensor at time t
//        virtual Velocity GetVelocity(double time) const;
//
//        /// Return the flow acceleration at the location of the sensor
//        virtual Acceleration GetAcceleration() const;
//
//        /// Return the flow acceleration at the location of the sensor at time t
//        virtual Acceleration GetAcceleration(double time) const;
//
//        void Initialize() override {};
//
//        void StepFinalize() override {};
//
//    };
//
//    // Forwward declaration
//    class FrLinearWaveField_;
//
//
//    /// Class to measure the flow velocity and pressure at a specific point.
//    /// Used when linear wave field is activated
//
//    class FrLinearFlowSensor_ : public FrFlowSensor_ {
//
//    private:
//        FrLinearWaveField_* m_waveField = nullptr;           ///< Linear wave field pointer
//
//        std::vector<chrono::ChVector<std::complex<double>>> m_steadyVelocity; ///< Steady state for effective time computation
//
//    public:
//        ~FrLinearFlowSensor_();
//
//        /// Constructor with wave field
//        explicit FrLinearFlowSensor_(FrWaveField_* waveField);
//
//        /// Constructor with 3D-coordinates
//        FrLinearFlowSensor_(FrWaveField_* waveField, double x, double y, double z);
//
////        FrLinearFlowSensor(FrWaveField* waveField, chrono::ChVector<> pos);
//
////        void SetWaveField(FrLinearWaveField* waveField);
//
////        FrLinearWaveField* GetWaveField() const override { return m_waveField; }
//
//        void Initialize() override;
//
//        Velocity GetVelocity(double time) const override;
//
//        /// Return the flow acceleration at the location of the sensor at time t
//        Acceleration GetAcceleration(double time) const override;
//
//        /// Return the flow velocity at the location of the sensor
//        Velocity GetVelocity() const override;
//
//        /// return the flow acceleration at the location of the sensor
//        Acceleration GetAcceleration() const override;
//
//    };



} // end of namespace frydom


#endif //FRYDOM_FRFLOWSENSOR_H
