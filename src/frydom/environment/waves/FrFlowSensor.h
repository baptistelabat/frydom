//
// Created by camille on 18/04/18.
//

#ifndef FRYDOM_FRFLOWSENSOR_H
#define FRYDOM_FRFLOWSENSOR_H

#include "frydom/core/FrConstants.h"
#include "frydom/core/FrObject.h"
#include "frydom/environment/waves/FrWaveField.h"
#include "frydom/utils/FrUtils.h"

namespace frydom {

    /// Class to measure the velocity and pressure of a flow at a specific point
    /// Useful when the position of the measurement is fixed in the global coordinate system
    /// to limit computation time.

    class FrFlowSensor : public FrObject {

    protected:
        double m_x = 0;         ///< Position of the sensor in global coordinate system
        double m_y = 0;         // FIXME : utiliser ChVector plutôt ?
        double m_z = 0;
        FrWaveField* m_waveField;       ///< Wave field defined in the system environment
        std::shared_ptr<FrRamp> m_waveRamp;         // FIXME : pourquoi la rampe est dans le capteur ?

    public:
        /// Default constructor
        FrFlowSensor() {};

        /// Constructor with position only
        FrFlowSensor(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {};

        /// Constructor with wave field
        FrFlowSensor(FrWaveField* waveField) { m_waveField = waveField; };

        /// Constructor operator with 3D-coordinates
        FrFlowSensor(FrWaveField* waveField, double x, double y, double z) : m_x(x), m_y(y), m_z(z) {
            m_waveField = waveField;
        }

        /// Constructor operator with position vector (3D)
        FrFlowSensor(chrono::ChVector<> pos);

        FrFlowSensor(FrWaveField* waveField, chrono::ChVector<> pos) : FrFlowSensor(pos) {
            m_waveField = waveField;
        }


        /// Define the x-coordinate in global coordinate system
        void SetX(const double x) { m_x = x; }
        double GetX() const { return m_x; }

        /// Define the y-coordinate in global coordinate system
        void SetY(const double y) { m_y = y; }
        double GetY() const { return m_y; }

        /// Define the z-coordinate in global coordinate system
        void SetZ(const double z) { m_z = z; }
        double GetZ() const { return m_z; }

        /// Define the vector position of the sensor in global coordinate system
        void SetPos(const chrono::ChVector<> pos);
        chrono::ChVector<>* GetPos() const;

        /// Return the wave field pointer (implemented in child class)
        virtual FrWaveField* GetWaveField() const {
            return m_waveField;
        }

        /// Return the flow velocity at the location of the sensor
        virtual chrono::ChVector<double> GetVelocity() const {
            return m_waveField->GetVelocity(m_x, m_y, m_z, true);
        }

        /// Return the flow velocity at the location of the sensor at time t
        virtual chrono::ChVector<double> GetVelocity(double time) const {
            m_waveField->Update(time);
            return this->GetVelocity();
        }

        /// Return the flow acceleration at the location of the sensor
        virtual chrono::ChVector<double> GetAcceleration() const {
            return m_waveField->GetAcceleration(m_x, m_y, m_z, true);
        }

        /// Return the flow acceleration at the location of the sensor at time t
        virtual chrono::ChVector<double> GetAcceleration(double time) const {
            m_waveField->Update(time);
            return this->GetAcceleration();
        }

        virtual void Initialize() override {};

        virtual void StepFinalize() override {};

    };

    /// Class to measure the flow velocity and pressure at a specific point.
    /// Used when linear wave field is activated

    class FrLinearFlowSensor : public FrFlowSensor {

    private:
        FrLinearWaveField* m_waveField = nullptr;           ///< Linear wave field pointer
        std::vector<chrono::ChVector<std::complex<double>>> m_steadyVelocity; ///< Steady state for effective time computation

    public:
        ~FrLinearFlowSensor() { delete m_waveField; }

        /// Constructor with wave field
        FrLinearFlowSensor(FrWaveField* waveField) {
            m_waveField = dynamic_cast<FrLinearWaveField*>(waveField);
        }

        /// Constructor with 3D-coordinates
        FrLinearFlowSensor(FrWaveField* waveField, double x, double y, double z)
                : FrFlowSensor(x, y, z) {
            m_waveField = dynamic_cast<FrLinearWaveField*>(waveField);
        }

        FrLinearFlowSensor(FrWaveField* waveField, chrono::ChVector<> pos) : FrFlowSensor(pos) {
            m_waveField = dynamic_cast<FrLinearWaveField*>(waveField);
        }

        void SetWaveField(FrLinearWaveField* waveField) { m_waveField = waveField; }

        FrLinearWaveField* GetWaveField() const override { return m_waveField; }

        void Initialize() override {
            m_steadyVelocity = m_waveField->GetSteadyVelocity(m_x, m_y, m_z);
        }

        chrono::ChVector<double> GetVelocity(double time) const override {

            auto emjwt = m_waveField->GetTimeCoeffs();

            chrono::ChVector<std::complex<double>> velocity(0.);
            for (unsigned int ifreq=0; ifreq<emjwt.size(); ++ifreq) {
                velocity += m_steadyVelocity[ifreq] * emjwt[ifreq];
            }

            chrono::ChVector<double> realVelocity = ChReal(velocity);

            auto waveRamp = m_waveField->GetWaveRamp();
            if (waveRamp && waveRamp->IsActive()) {
                m_waveField->GetWaveRamp()->Apply(m_waveField->GetTime(),realVelocity);
            }

            return realVelocity;

        }

        /// Return the flow acceleration at the location of the sensor at time t
        chrono::ChVector<double> GetAcceleration(double time) const override {

            auto emjwt_dt = m_waveField->GetTimeCoeffsDt();

            chrono::ChVector<std::complex<double>> acceleration(0.);
            for (unsigned int ifreq=0; ifreq<emjwt_dt.size(); ++ifreq) {
                acceleration += m_steadyVelocity[ifreq] * emjwt_dt[ifreq];
            }

            chrono::ChVector<double> realAcceleration = ChReal(acceleration);

            auto waveRamp = m_waveField->GetWaveRamp();
            if (waveRamp && waveRamp->IsActive()) {
                m_waveField->GetWaveRamp()->Apply(m_waveField->GetTime(), realAcceleration);
            }

            return realAcceleration;

        }

        /// Return the flow velocity at the location of the sensor
        chrono::ChVector<double> GetVelocity() const override {
            // TODO
        }

        /// return the flow acceleration at the location of the sensor
        chrono::ChVector<double> GetAcceleration() const override {
            // TODO
        }

    };



} // end of namespace frydom


#endif //FRYDOM_FRFLOWSENSOR_H
