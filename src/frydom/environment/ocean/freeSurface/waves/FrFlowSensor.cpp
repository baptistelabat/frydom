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


#include "FrFlowSensor.h"

#include "FrWaveField.h"
#include "frydom/utils/FrUtils.h"




namespace frydom {

//    FrFlowSensor::FrFlowSensor() {}

//    FrFlowSensor::FrFlowSensor(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {}

    FrFlowSensor::FrFlowSensor(FrWaveField *waveField) : m_waveField(waveField) {}

    FrFlowSensor::FrFlowSensor(FrWaveField *waveField, double x, double y, double z) :
                m_waveField(waveField), m_x(x), m_y(y), m_z(z) {}

//    FrFlowSensor::FrFlowSensor(const chrono::ChVector<> pos) {
//        m_x = pos.x();
//        m_y = pos.y();
//        m_z = pos.y();
//    }

//    FrFlowSensor::FrFlowSensor(FrWaveField *waveField, chrono::ChVector<> pos) : FrFlowSensor(pos) {
//        m_waveField = waveField;
//    }

//    void FrFlowSensor::SetX(const double x) { m_x = x; }
//
//    double FrFlowSensor::GetX() const { return m_x; }
//
//    void FrFlowSensor::SetY(const double y) { m_y = y; }
//
//    double FrFlowSensor::GetY() const { return m_y; }
//
//    void FrFlowSensor::SetZ(const double z) { m_z = z; }
//
//    double FrFlowSensor::GetZ() const { return m_z; }

    void FrFlowSensor::SetPos(double x, double y, double z) {
        m_x = x;
        m_y = y;
        m_z = z;
    }

//    chrono::ChVector<>* FrFlowSensor::GetPos() const {
//        return new chrono::ChVector<double>(m_x, m_y, m_z);
//    }

    void FrFlowSensor::GetPos(double &x, double &y, double &z) {
        x = m_x;
        y = m_y;
        z = m_z;
    }

//    FrWaveField *FrFlowSensor::GetWaveField() const {
//        return m_waveField;
//    }

    chrono::ChVector<double> FrFlowSensor::GetVelocity() const {
        return m_waveField->GetVelocity(m_x, m_y, m_z, true);
    }

    chrono::ChVector<double> FrFlowSensor::GetVelocity(double time) const {
        m_waveField->Update(time);
        return this->GetVelocity();
    }

    chrono::ChVector<double> FrFlowSensor::GetAcceleration() const {
        return m_waveField->GetAcceleration(m_x, m_y, m_z, true);
    }

    chrono::ChVector<double> FrFlowSensor::GetAcceleration(double time) const {
        m_waveField->Update(time);
        return this->GetAcceleration();
    }



    FrLinearFlowSensor::~FrLinearFlowSensor() { delete m_waveField; } // TODO : voir si on a besoin du delete !

    FrLinearFlowSensor::FrLinearFlowSensor(FrLinearWaveField *waveField) : FrFlowSensor(waveField) {}

    FrLinearFlowSensor::FrLinearFlowSensor(FrWaveField *waveField, double x, double y, double z)
            : FrFlowSensor(waveField, x, y, z) {}

//    FrLinearFlowSensor::FrLinearFlowSensor(FrWaveField *waveField, chrono::ChVector<> pos) : FrFlowSensor(pos) {
//        m_waveField = dynamic_cast<FrLinearWaveField*>(waveField);
//    }

//    void FrLinearFlowSensor::SetWaveField(FrLinearWaveField *waveField) { m_waveField = waveField; }

    FrLinearWaveField* FrLinearFlowSensor::GetWaveField() const { return m_waveField; }

    void FrLinearFlowSensor::Initialize() {
        m_steadyVelocity = m_waveField->GetSteadyVelocity(m_x, m_y, m_z);
    }

    chrono::ChVector<double> FrLinearFlowSensor::GetVelocity(double time) const {

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

    chrono::ChVector<double> FrLinearFlowSensor::GetAcceleration(double time) const {

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

    chrono::ChVector<double> FrLinearFlowSensor::GetVelocity() const {
        // TODO
    }

    chrono::ChVector<double> FrLinearFlowSensor::GetAcceleration() const {
        // TODO
    }



















    // REFACTORING --------------->>>>>>>>>>>>>>>

//
//
//
//
//
//    FrFlowSensor_::FrFlowSensor_(FrWaveField_ *waveField) : m_waveField(waveField) {}
//
//    FrFlowSensor_::FrFlowSensor_(FrWaveField_ *waveField, double x, double y, double z) :
//        m_waveField(waveField), m_x(x), m_y(y), m_z(z) {}
//
////    void FrFlowSensor_::SetX(const double x) { m_x = x; }
////
////    double FrFlowSensor_::GetX() const { return m_x; }
////
////    void FrFlowSensor_::SetY(const double y) { m_y = y; }
////
////    double FrFlowSensor_::GetY() const { return m_y; }
////
////    void FrFlowSensor_::SetZ(const double z) { m_z = z; }
////
////    double FrFlowSensor_::GetZ() const { return m_z; }
//
//    void FrFlowSensor_::SetPos(double x, double y, double z) {
//        m_x = x;
//        m_y = y;
//        m_z = z;
//    }
//
//    void FrFlowSensor_::GetPos(double& x, double& y, double& z) const {
//        x = m_x;
//        y = m_y;
//        z = m_z;
//    }
//
//    FrWaveField_* FrFlowSensor_::GetWaveField() const {
//        return m_waveField;
//    }
//
//    Velocity FrFlowSensor_::GetVelocity() const {
//        return m_waveField->GetVelocity(m_x, m_y, m_z, true);
//    }
//
//    Velocity FrFlowSensor_::GetVelocity(double time) const {
//        m_waveField->Update(time);
//        return this->GetVelocity();
//    }
//
//    Acceleration FrFlowSensor_::GetAcceleration() const {
//        return m_waveField->GetAcceleration(m_x, m_y, m_z, true);
//    }
//
//    Acceleration FrFlowSensor_::GetAcceleration(double time) const {
//        m_waveField->Update(time);
//        return this->GetAcceleration();
//    }
//
//
//
//
//    FrLinearFlowSensor_::~FrLinearFlowSensor_() { delete m_waveField; } // TODO: voir pourquoi le delete
//
//    FrLinearFlowSensor_::FrLinearFlowSensor_(FrWaveField_ *waveField) : FrFlowSensor_(waveField) {}
//
//    FrLinearFlowSensor_::FrLinearFlowSensor_(FrWaveField_ *waveField, double x, double y, double z)
//            : FrFlowSensor_(waveField, x, y, z) {}
//
////    FrLinearFlowSensor::FrLinearFlowSensor(FrWaveField *waveField, chrono::ChVector<> pos) : FrFlowSensor(pos) {
////        m_waveField = dynamic_cast<FrLinearWaveField*>(waveField);
////    }
//
////    void FrLinearFlowSensor::SetWaveField(FrLinearWaveField *waveField) { m_waveField = waveField; }
//
//    void FrLinearFlowSensor_::Initialize() {
//        m_steadyVelocity = m_waveField->GetSteadyVelocity(m_x, m_y, m_z);
//    }
//
//    Velocity FrLinearFlowSensor_::GetVelocity(double time) const {
//
////        auto emjwt = m_waveField->GetTimeCoeffs();
////
////        chrono::ChVector<std::complex<double>> velocity(0.);
////        for (unsigned int ifreq=0; ifreq<emjwt.size(); ++ifreq) {
////            velocity += m_steadyVelocity[ifreq] * emjwt[ifreq];
////        }
////
////        chrono::ChVector<double> realVelocity = ChReal(velocity);
////
////        auto waveRamp = m_waveField->GetWaveRamp();
////        if (waveRamp && waveRamp->IsActive()) {
////            m_waveField->GetWaveRamp()->Apply(m_waveField->GetTime(),realVelocity);
////        }
////
////        return realVelocity;
//
//    }
//
//    Acceleration FrLinearFlowSensor_::GetAcceleration(double time) const {
//
////        auto emjwt_dt = m_waveField->GetTimeCoeffsDt();
////
////        chrono::ChVector<std::complex<double>> acceleration(0.);
////        for (unsigned int ifreq=0; ifreq<emjwt_dt.size(); ++ifreq) {
////            acceleration += m_steadyVelocity[ifreq] * emjwt_dt[ifreq];
////        }
////
////        chrono::ChVector<double> realAcceleration = ChReal(acceleration);
////
////        auto waveRamp = m_waveField->GetWaveRamp();
////        if (waveRamp && waveRamp->IsActive()) {
////            m_waveField->GetWaveRamp()->Apply(m_waveField->GetTime(), realAcceleration);
////        }
////
////        return realAcceleration;
//
//    }
//
//    Velocity FrLinearFlowSensor_::GetVelocity() const {
//        // TODO
//    }
//
//    Acceleration FrLinearFlowSensor_::GetAcceleration() const {
//        // TODO
//    }
}
