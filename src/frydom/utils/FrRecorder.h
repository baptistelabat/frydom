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


#ifndef FRYDOM_FRTIMERINGBUFFER_H
#define FRYDOM_FRTIMERINGBUFFER_H

#include <vector>
#include <cassert>
#include <deque>
#include <cfloat>
#include <iostream>

#include "boost/circular_buffer.hpp"

//#include "chrono/core/ChVector.h"


namespace frydom {

    /**
     * \class FrRecorder
     * \brief Class for recording velocities used in the convolution integral.
     */
    template <class T>
    class FrRecorder {
    private:

        double m_timePersistence = 1800; // Default is 0.5 hour persistence...

        std::deque<std::pair<double, T>> m_data;

        unsigned int m_damping = 50;
        unsigned int m_iter = 0;

    public:
        FrRecorder() {
            m_data.push_front(std::pair<double, T>(0., T()));
        };

        explicit FrRecorder(double timePersistence) : m_timePersistence(timePersistence) {}

        void SetTimePersistence(const double timePersistence) {
            m_timePersistence = timePersistence;
        }

        double GetTimePersistence() const { return m_timePersistence; }

        void SetDamping(unsigned int damping) { m_damping = damping; }

        unsigned int GetDamping() const { return m_damping; }

        double GetRecordedDuration() const {
            return m_data.front().first - m_data.back().first;
        }

        void AdjustData() {

            auto lastTime = m_data.front().first;
            double oldestTime = m_data.at(m_data.size()-2).first;
            while ((lastTime - oldestTime) > m_timePersistence) {
                m_data.pop_back();
                oldestTime = m_data.at(m_data.size() - 2).first;
            }

        }

        void record(const double time, const T& data) {
            // TODO: faire attention de ne pas ajouter plusieurs fois le meme temps (solveur multipas...)
            // Si on a deja le temps donne, alors on remplace

            auto pair = std::pair<double, T>(time, data);

            // Get the last time entry
            auto lastTime = m_data.front().first;

            if (time < lastTime) {
                return;
            }

            if (time == lastTime) {
                // We replace the last element
                m_data.front() = pair;
            } else {
                m_data.push_front(pair);
            }

            // Truncate the recorded data if they are longer than the persistence duration
            // TODO: ne le faire qu'avec un amortissement
            if (m_iter == m_damping) {
                AdjustData();
                m_iter = 0;
            }

            std::cout << GetRecordedDuration() << std::endl;

            m_iter++;

        }


    };




















    // >>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    /**
     * \class FrRecorder_
     * \brief Class for recording velocities used in the convolution integral.
     */
    template <class T>
    class FrRecorder_ {

    private:

        unsigned int m_size = 0;
        unsigned int m_step = 0;

        boost::circular_buffer<std::pair<double, T>> m_data;

    public:



    };


    /**
     * \class FrTimeRecorder_
     * \brief Class for recording the time.
     */
    template <class T>
    class FrTimeRecorder_ {


    private:

        double m_timePersistence = 1800.;
        double m_timeStep;

        unsigned int m_size = 0;
        unsigned int m_nstep = 0;

        double m_lastTime = 0.;
        double m_prevTime = 0.;
        double m_deltaTime = 0.;

        boost::circular_buffer<T> m_data;

    public:

        FrTimeRecorder_() = default;

        explicit FrTimeRecorder_(double timePersistence) : m_timePersistence(timePersistence) { }

        explicit FrTimeRecorder_(double timePersistence, double timeStep)
            : m_timePersistence(timePersistence), m_timeStep(timeStep) { }

        void SetTimePersistence(const double timePersistence);

        double GetTimePersistence() const;

        double GetRecordDuration() const;

        void SetTimeStep(double timeStep);

        double GetTimeStep() const;

        void Record(const double time, const T& data);

        void Initialize();

        boost::circular_buffer<T> GetData() const;

        std::vector<double> GetTime() const;

        double GetLastTime() const;

        T GetMean() const;

    private:

        T Interpolate(const double time, const T& data, const double& lastTime);

    };


    template <class T>
    void FrTimeRecorder_<T>::SetTimePersistence(const double timePersistence) {
        assert(timePersistence > DBL_EPSILON);
        m_timePersistence = timePersistence;
    }

    template <class T>
    double FrTimeRecorder_<T>::GetTimePersistence() const {
        return m_timePersistence;
    }

    template <class T>
    double FrTimeRecorder_<T>::GetRecordDuration() const {
        // TODO
    }

    template <class T>
    void FrTimeRecorder_<T>::SetTimeStep(double timeStep) {
        assert(timeStep > DBL_EPSILON);
        m_timeStep = timeStep;
    }

    template <class T>
    double FrTimeRecorder_<T>::GetTimeStep() const {
        return m_timeStep;
    }

    template <class T>
    void FrTimeRecorder_<T>::Initialize() {
        m_size = int(m_timePersistence / m_timeStep);
        m_data.set_capacity(m_size);
    }

    template <class T>
    void FrTimeRecorder_<T>::Record(const double time, const T &data) {

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
    T FrTimeRecorder_<T>::Interpolate(const double intTime, const T& data, const double& time) {

        auto data_prev = m_data.front();
        auto a = (data - data_prev)/(time - m_lastTime);
        auto b = (data_prev * time - data * m_lastTime)/(time - m_lastTime);

        return a*intTime + b;
    }


    template <class T>
    boost::circular_buffer<T> FrTimeRecorder_<T>::GetData() const { return m_data; }

    template <class T>
    std::vector<double> FrTimeRecorder_<T>::GetTime() const {

        std::vector<double> vtime;

        vtime.push_back(0.);

        if (m_deltaTime > FLT_EPSILON && m_data.size() > 1) {
            for (int i = 0; i < m_data.size() - 1; ++i) {
                vtime.push_back(m_deltaTime + i * m_timeStep);
            }
        }

        return vtime;
    }

    template <class T>
    double FrTimeRecorder_<T>::GetLastTime() const { return m_lastTime; }


    template <class T>
    T FrTimeRecorder_<T>::GetMean() const {

        auto result = T();
        for (auto val: m_data) {
            result += val;
        }

        if (m_data.size() > 0) { result /= m_data.size(); }

        return result;
    }

}  // end namespace frydom


#endif //FRYDOM_FRTIMERINGBUFFER_H
