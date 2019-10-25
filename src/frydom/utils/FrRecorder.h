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
   * \brief This class is used to record arbitrary data at the different time instant of the simulation
   * Data are return in the reverse time order from the last stored data.
   */
  template<class T>
  class FrTimeRecorder {

   private:

    double m_timePersistence = 1800.;       ///< Maximum time-length of the recorder in seconds
    double m_timeStep = 0.01;               ///< Time step between each records

    unsigned int m_size = 0;                ///< Number of elements in the recorder

    double m_lastTime = 0.;                 ///< Current time in simulation of the last record
    double m_prevTime = 0.;                 ///< Time in simulation of the penultimate record
    double m_deltaTime = 0.;                ///< Elapsed time between the two last records

    boost::circular_buffer<T> m_data;       ///< Buffer with the recordered data

   public:

    /// Default constructor of the time recorder
    FrTimeRecorder() = default;

    /// Constructor of the time recorder with specified time persistence
    /// \param timePersistence Maximum time-length of the recorder in seconds
    explicit FrTimeRecorder(double timePersistence) : m_timePersistence(timePersistence) {}

    /// Constructor of the time recorder with specified time persistence and time-step
    /// \param timePersistence Maximum time-length of the recorder in seconds
    /// \param timeStep Time step between each records
    explicit FrTimeRecorder(double timePersistence, double timeStep)
        : m_timePersistence(timePersistence), m_timeStep(timeStep) {}

    /// Set the total time-length of the recorder in seconds
    /// \param timePersistence Total time-length of the recorder in second
    void SetTimePersistence(const double timePersistence);

    /// Get the maximum time-length of the recorder in seconds
    /// \return Maximum time-length of the recorder in seconds
    double GetTimePersistence() const;

    /// Effective time-length of the recorder in seconds
    /// \return Effective time-length of the recorder in seconds
    double GetRecordDuration() const;

    /// Set the time-step of the recorder in seconds
    /// \param timeStep Time-step of the recorder in seconds
    void SetTimeStep(double timeStep);

    /// Get the time-step of the recorder in seconds
    /// \return Time-step of the recorder in seconds
    double GetTimeStep() const;

    /// Recordering data
    /// \param time Current time of the simulation, in seconds
    /// \param data Data to be recordered
    void Record(const double time, const T &data);

    /// Initialize the size of the recorder
    void Initialize();

    /// Clear stored data
    void Clear();

    /// Return the stored data vector
    /// \return Recorder data vector
    boost::circular_buffer<T> GetData() const;

    /// Get the data interpolated at a specific time delay from the last time stored in the recorder
    /// \param time Time delay from the last time stored in the recorder
    /// \return Data interpolated at the specific time
    T GetData(double time) const;

    /// Get the time vector corresponding to the recordered data in the recorder
    std::vector<double> GetTime() const;

    /// Get the last time stored in the recorder
    double GetLastTime() const;

    /// Get the mean value of the stored data
    T GetMean() const;

   private:

    /// Interpolate data bewteen the penultimate stored data and the new data at a given time
    /// \param time Target time for the interpolation
    /// \param data Data value at the current time
    /// \param lastTime Current time
    /// \return Interpolated data value at the target time
    T Interpolate(const double time, const T &data, const double &lastTime);

  };


  template<class T>
  void FrTimeRecorder<T>::SetTimePersistence(const double timePersistence) {
    assert(timePersistence > DBL_EPSILON);
    m_timePersistence = timePersistence;
  }

  template<class T>
  double FrTimeRecorder<T>::GetTimePersistence() const {
    return m_timePersistence;
  }

  template<class T>
  double FrTimeRecorder<T>::GetRecordDuration() const {
    return 0.;
    // TODO
  }

  template<class T>
  void FrTimeRecorder<T>::SetTimeStep(double timeStep) {
    assert(timeStep > DBL_EPSILON);
    m_timeStep = timeStep;
  }

  template<class T>
  double FrTimeRecorder<T>::GetTimeStep() const {
    return m_timeStep;
  }

  template<class T>
  void FrTimeRecorder<T>::Initialize() {
    m_size = int(m_timePersistence / m_timeStep);
    m_data.set_capacity(m_size);
  }

  template<class T>
  void FrTimeRecorder<T>::Clear() {
    m_data.clear();
    Initialize();
  }

  template<class T>
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

  template<class T>
  T FrTimeRecorder<T>::GetData(double time) const {
    auto vtime = this->GetTime();
    auto upper_time = std::lower_bound(vtime.begin(), vtime.end(), time);
    auto it = std::distance(vtime.begin(), upper_time) - 1;
    assert(it < vtime.size());
    auto c0 = (vtime[it + 1] - time) / (vtime[it + 1] - vtime[it]);
    auto c1 = (time - vtime[it]) / (vtime[it + 1] - vtime[it]);
    return m_data[it] * c0 + m_data[it + 1] * c1;
  }

  template<class T>
  T FrTimeRecorder<T>::Interpolate(const double intTime, const T &data, const double &time) {

    auto data_prev = m_data.front();
    auto a = (data - data_prev) / (time - m_lastTime);
    auto b = (data_prev * time - data * m_lastTime) / (time - m_lastTime);

    return a * intTime + b;
  }


  template<class T>
  boost::circular_buffer<T> FrTimeRecorder<T>::GetData() const { return m_data; }

  template<class T>
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

  template<class T>
  double FrTimeRecorder<T>::GetLastTime() const { return m_lastTime; }


  template<class T>
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
