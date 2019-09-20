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


#include "chrono/core/ChMatrixDynamic.h"  // TODO : voir pourquoi on est oblige d'inclure ca...
#include "chrono/core/ChFrame.h"

#include "FrTidalModel.h"


namespace frydom {

    void FrUTCTime::Check() {
      assert(m_hours > 0. && m_hours < 24);
      assert(m_minutes > 0. && m_minutes <= 60.);
      assert(m_seconds > 0. && m_seconds <= 60.);
      assert(m_local_correction > -24 && m_local_correction < 24);  // TODO verifier
    }

    FrUTCTime::FrUTCTime(const unsigned int hours, const unsigned int minutes, const unsigned int seconds) :
        m_hours(hours),
        m_minutes(minutes),
        m_seconds(seconds) {
      Check();
    }

    FrUTCTime::FrUTCTime(const unsigned int hours, const unsigned int minutes) :
        m_hours(hours),
        m_minutes(minutes),
        m_seconds(0) {
      Check();
    }

    double FrUTCTime::GetHours() const {
      return (double) m_hours + (double) m_minutes / 60. + (double) m_seconds / 3600.;
    }

    double FrUTCTime::GetMinutes() const {
      return (double) m_hours * 60 + (double) m_minutes + (double) m_seconds / 60.;
    }

    double FrUTCTime::GetSeconds() const {
      return (double) m_hours * 3600 + (double) m_minutes * 60 + (double) m_seconds;
    }


    template<typename OffshoreSystemType>
    void FrTidal<OffshoreSystemType>::BuildTable() {
      switch (m_mode) {
        case NO_TIDAL:
          return;
        case TWELFTH_RULE:
          BuildTwelfthRuleTable();
      }
    }

    template<typename OffshoreSystemType>
    void FrTidal<OffshoreSystemType>::BuildTwelfthRuleTable() {
      double h_twelfth = fabs(m_h2 - m_h1) / 12.; // One twelfth

      double t1 = m_t1.GetMinutes();
      double t2 = m_t2.GetMinutes();

      double dt = (t2 - t1) / 6.;

      std::vector<double> timeVect;
      timeVect.reserve(7);
      for (uint it = 0; it < 7; ++it) {
        timeVect.push_back(t1 + it * dt);  // TODO: verifier que la derniere valeur est egale a t2 !!
      }

      std::vector<double> hVect;
      hVect.reserve(13);

      int op;
      if (m_level1 == LOW) {
        op = 1;
      } else {
        op = -1;
      }

      hVect.push_back(m_h1);  // Last extremum tidal level
      hVect.push_back(m_h1 + op * 1 * h_twelfth); // 1 twelfth
      hVect.push_back(m_h1 + op * 3 * h_twelfth); // 2 twelfth
      hVect.push_back(m_h1 + op * 6 * h_twelfth); // 3 twelfth
      hVect.push_back(m_h1 + op * 9 * h_twelfth); // 3 twelfth
      hVect.push_back(m_h1 + op * 11 * h_twelfth); // 2 twelfth
      hVect.push_back(m_h1 + op * 12 * h_twelfth); // 1 twelfth // TODO: verifier qu'on a h2...

      // Populating the interpolation table
      tidalTable.SetX(timeVect);
      tidalTable.AddY("tidal_height", hVect);
    }

    template<typename OffshoreSystemType>
    FrTidal<OffshoreSystemType>::FrTidal(FrFreeSurface<OffshoreSystemType> *freeSurface) : m_freeSurface(freeSurface) {
      m_tidalFrame = std::make_unique<chrono::ChFrame<double>>();
    }

    template<typename OffshoreSystemType>
    FrTidal<OffshoreSystemType>::FrTidal(FrFreeSurface<OffshoreSystemType> *freeSurface,
                                         const FrUTCTime t1, const double h1,
                                         FrTidal<OffshoreSystemType>::TIDAL_LEVEL level1, const FrUTCTime t2,
                                         const double h2, FrTidal<OffshoreSystemType>::TIDAL_LEVEL level2) :
        m_t1(t1),
        m_h1(h1),
        m_level1(level1),
        m_t2(t2),
        m_h2(h2),
        m_level2(level2),
        m_mode(TWELFTH_RULE),
        m_freeSurface(freeSurface) {

      assert(h1 >= 0. && h2 >= 0.);
      assert(level1 != level2);  // Levels have to be different
      assert(t2.GetSeconds() > t1.GetSeconds());  // Ajouter operateur de comparaison de temps

      BuildTable();

    }

    template<typename OffshoreSystemType>
    FrTidal<OffshoreSystemType>::~FrTidal() = default;

    template<typename OffshoreSystemType>
    void FrTidal<OffshoreSystemType>::Update(const double time) {
      double waterHeight = 0.;

      if (m_mode == NO_TIDAL) {
        waterHeight = m_h1;
      }

      if (m_mode == TWELFTH_RULE) {
        waterHeight = tidalTable.Eval("tidal_height", m_time);
      }

      m_tidalFrame->GetPos().z() = waterHeight;
    }

    template<typename OffshoreSystemType>
    const double FrTidal<OffshoreSystemType>::GetHeight(FRAME_CONVENTION fc) const {
      double ZPos = m_tidalFrame->GetPos().z();
      if (IsNED(fc)) { ZPos = -ZPos; }
      return ZPos;
    }

    template<typename OffshoreSystemType>
    const chrono::ChFrame<double> *FrTidal<OffshoreSystemType>::GetTidalFrame() const {
      return m_tidalFrame.get();
    }

    template<typename OffshoreSystemType>
    void FrTidal<OffshoreSystemType>::Initialize() {

    }

    template<typename OffshoreSystemType>
    void FrTidal<OffshoreSystemType>::StepFinalize() {

    }

    template<typename OffshoreSystemType>
    void FrTidal<OffshoreSystemType>::SetNoTidal() { m_mode = NO_TIDAL; }

}  // end namespace frydom
