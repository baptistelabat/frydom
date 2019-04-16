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


#ifndef FRYDOM_FRTIMERINGBUFFER_H
#define FRYDOM_FRTIMERINGBUFFER_H

#include <vector>
#include <cfloat>

#include "boost/circular_buffer.hpp"


namespace frydom {

    /**
     * \class FrTimeRecorder
     * \brief Class for recording the time.
     */
    template <class T>
    class FrTimeRecorder {


    private:

        double m_timePersistence = 1800.;
        double m_timeStep = 0.01;

        unsigned int m_size = 0;
        unsigned int m_nstep = 0;

        double m_lastTime = 0.;
        double m_prevTime = 0.;
        double m_deltaTime = 0.;

        boost::circular_buffer<T> m_data;

    public:

        FrTimeRecorder() = default;

        explicit FrTimeRecorder(double timePersistence) : m_timePersistence(timePersistence) { }

        explicit FrTimeRecorder(double timePersistence, double timeStep)
            : m_timePersistence(timePersistence), m_timeStep(timeStep) { }

        void SetTimePersistence(const double timePersistence);

        double GetTimePersistence() const;

        double GetRecordDuration() const;

        void SetTimeStep(double timeStep);

        double GetTimeStep() const;

        void Record(const double time, const T& data);

        void Initialize();

        void Clear();

        boost::circular_buffer<T> GetData() const;

        std::vector<double> GetTime() const;

        double GetLastTime() const;

        T GetMean() const;

    private:

        T Interpolate(const double time, const T& data, const double& lastTime);

    };


    template <class T>
    void FrTimeRecorder<T>::SetTimePersistence(const double timePersistence) {
        assert(timePersistence > DBL_EPSILON);
        m_timePersistence = timePersistence;
    }

    template <class T>
    double FrTimeRecorder<T>::GetTimePersistence() const {
        return m_timePersistence;
    }

    template <class T>
    double FrTimeRecorder<T>::GetRecordDuration() const {
        // TODO
    }

    template <class T>
    void FrTimeRecorder<T>::SetTimeStep(double timeStep) {
        assert(timeStep > DBL_EPSILON);
        m_timeStep = timeStep;
    }

    template <class T>
    double FrTimeRecorder<T>::GetTimeStep() const {
        return m_timeStep;
    }

    template <class T>
    void FrTimeRecorder<T>::Initialize() {
        m_size = int(m_timePersistence / m_timeStep);
        m_data.set_capacity(m_size);
    }

    template <class T>
    void FrTimeRecorder<T>::Clear() {
        m_data.clear();
        Initialize();
    }

    template <class T>
    void FrTimeRecorder<T>::Record(const double time, const T &data) {

        if (m_data.empty()) {
            m_data.push_front(data);
            m_lastTime = time;
            m_prevTime = time;
            return;
        }

        if (time == m_lastTime) return;

        m_deltaTime = time - m_prevTime;

        if (m_deltaTime < m_timeStep) {
            m_data.front() = data;
            m_lastTime = time;
        } else {
            auto new_data = Interpolate(m_prevTime + m_timeStep, data, time);
            m_data.front() = new_data;
            m_data.push_front(data);
            m_prevTime += m_timeStep;
            m_lastTime = time;
            m_deltaTime = m_lastTime - m_prevTime;
        }
    }

    template <class T>
    T FrTimeRecorder<T>::Interpolate(const double intTime, const T& data, const double& time) {

        auto data_prev = m_data.front();
        auto a = (data - data_prev)/(time - m_lastTime);
        auto b = (data_prev * time - data * m_lastTime)/(time - m_lastTime);

        return a*intTime + b;
    }


    template <class T>
    boost::circular_buffer<T> FrTimeRecorder<T>::GetData() const { return m_data; }

    template <class T>
    std::vector<double> FrTimeRecorder<T>::GetTime() const {

        std::vector<double> vtime;

        vtime.push_back(0.);

        //if (m_deltaTime > FLT_EPSILON && m_data.size() > 1) {
        if (m_data.size() > 1) {
            vtime.push_back(m_deltaTime);
        }

        if (m_data.size() > 1) {
            for (int i = 1; i < m_data.size() - 1; ++i) {
                vtime.push_back(m_deltaTime + i * m_timeStep);
            }
        }

        return vtime;
    }

    template <class T>
    double FrTimeRecorder<T>::GetLastTime() const { return m_lastTime; }


    template <class T>
    T FrTimeRecorder<T>::GetMean() const {

        auto result = T();
        for (auto val: m_data) {
            result += val;
        }

        if (m_data.size() > 0) { result /= m_data.size(); }

        return result;
    }

}  // end namespace frydom


#endif //FRYDOM_FRTIMERINGBUFFER_H
