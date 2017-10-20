//
// Created by frongere on 19/10/17.
//

#ifndef FRYDOM_FRTIMERINGBUFFER_H
#define FRYDOM_FRTIMERINGBUFFER_H

#include <vector>
#include <cassert>
#include <deque>
#include <iostream>

//#include "chrono/core/ChVector.h"


namespace frydom {

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




}  // end namespace frydom


#endif //FRYDOM_FRTIMERINGBUFFER_H
