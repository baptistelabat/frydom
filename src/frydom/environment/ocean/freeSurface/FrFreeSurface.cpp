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


#include "FrFreeSurface.h"

#include "frydom/asset/FrFreeSurfaceGridAsset.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrFreeSurface<OffshoreSystemType>::~FrFreeSurface() = default;

    template<typename OffshoreSystemType>
    FrFreeSurface<OffshoreSystemType>::FrFreeSurface(FrOcean<OffshoreSystemType> *ocean) : m_ocean(ocean) {

      // Creating a waveField and a tidal model
      m_waveField = std::make_unique<FrNullWaveField>(this);
      m_tidal = std::make_unique<FrTidal>(this);
      m_freeSurfaceGridAsset = std::make_shared<FrFreeSurfaceGridAsset>(this);

    }

    template<typename OffshoreSystemType>
    FrAtmosphere<OffshoreSystemType> *
    FrFreeSurface<OffshoreSystemType>::GetAtmosphere() const { return m_ocean->GetEnvironment()->GetAtmosphere(); }

    template<typename OffshoreSystemType>
    FrOcean<OffshoreSystemType> *FrFreeSurface<OffshoreSystemType>::GetOcean() const { return m_ocean; }

    template<typename OffshoreSystemType>
    FrTidal<OffshoreSystemType> *FrFreeSurface<OffshoreSystemType>::GetTidal() const { return m_tidal.get(); }

    template<typename OffshoreSystemType>
    FrWaveField<OffshoreSystemType> *
    FrFreeSurface<OffshoreSystemType>::GetWaveField() const { return m_waveField.get(); }

    template<typename OffshoreSystemType>
    double FrFreeSurface<OffshoreSystemType>::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
      m_waveField->GetElevation(x, y, fc);
    }

    template<typename OffshoreSystemType>
    FrFreeSurfaceGridAsset<OffshoreSystemType> *
    FrFreeSurface<OffshoreSystemType>::GetFreeSurfaceGridAsset() const { return m_freeSurfaceGridAsset.get(); }

    template<typename OffshoreSystemType>
    double FrFreeSurface<OffshoreSystemType>::GetPosition(FRAME_CONVENTION fc) const {
      return GetPosition(0., 0., fc);
    }

    template<typename OffshoreSystemType>
    double FrFreeSurface<OffshoreSystemType>::GetPosition(double x, double y, FRAME_CONVENTION fc) const {
      return m_tidal->GetHeight(fc) + m_waveField->GetElevation(x, y, fc);
    }

    template<typename OffshoreSystemType>
    double FrFreeSurface<OffshoreSystemType>::GetPosition(const Position worldPos, FRAME_CONVENTION fc) const {
      return GetPosition(worldPos[0], worldPos[1], fc);
    }

    template<typename OffshoreSystemType>
    void FrFreeSurface<OffshoreSystemType>::GetPosition(Position &worldPos, FRAME_CONVENTION fc) const {
      worldPos[2] = GetPosition(worldPos[0], worldPos[1], fc);
    }

    template<typename OffshoreSystemType>
    double FrFreeSurface<OffshoreSystemType>::GetPressure(double x, double y, double z, FRAME_CONVENTION fc) const {

      // This function computes the pressure.

      return m_waveField->GetPressure(x, y, z, fc);

    }


//    void FrFreeSurface<OffshoreSystemType>::NoWaves() {
//        m_waveField = std::make_unique<FrNullWaveField>(this);
//    }
//
//    FrAiryRegularWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryRegularWaveField() {
//        m_waveField = std::make_unique<FrAiryRegularWaveField>(this);
//        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
//    }
//
//    FrAiryRegularWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryRegularWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
//                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
//        auto waveField = SetAiryRegularWaveField();
//        waveField->SetWaveHeight(waveHeight);
//        waveField->SetWavePeriod(wavePeriod);
//        waveField->SetDirection(waveDirAngle, unit, fc, dc);
//        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
//    }
//
//
//    FrAiryRegularWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryRegularWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
//                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
//        auto waveField = SetAiryRegularWaveField();
//        waveField->SetWaveHeight(waveHeight);
//        waveField->SetWavePeriod(wavePeriod);
//        waveField->SetDirection(waveDirection, fc, dc);
//        return waveField;
//    }
//
//
//    FrAiryRegularOptimWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryRegularOptimWaveField() {
//        m_waveField = std::make_unique<FrAiryRegularOptimWaveField>(this);
//        return dynamic_cast<FrAiryRegularOptimWaveField*>(m_waveField.get());
//    }
//
//    FrAiryRegularOptimWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
//                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
//        auto waveField = SetAiryRegularOptimWaveField();
//        waveField->SetWaveHeight(waveHeight);
//        waveField->SetWavePeriod(wavePeriod);
//        waveField->SetDirection(waveDirAngle, unit, fc, dc);
//        return waveField;
//    }
//
//    FrAiryRegularOptimWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
//                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
//        auto waveField = SetAiryRegularOptimWaveField();
//        waveField->SetWaveHeight(waveHeight);
//        waveField->SetWavePeriod(wavePeriod);
//        waveField->SetDirection(waveDirection, fc, dc);
//        return waveField;
//    }
//
//
//    FrAiryIrregularWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryIrregularWaveField() {
//        m_waveField = std::make_unique<FrAiryIrregularWaveField>(this);
//        return dynamic_cast<FrAiryIrregularWaveField*>(m_waveField.get());
//    }
//
//    FrAiryIrregularOptimWaveField*
//    FrFreeSurface<OffshoreSystemType>::SetAiryIrregularOptimWaveField() {
//        m_waveField = std::make_unique<FrAiryIrregularOptimWaveField>(this);
//        return dynamic_cast<FrAiryIrregularOptimWaveField*>(m_waveField.get());
//    }
    template<typename OffshoreSystemType>
    void FrFreeSurface<OffshoreSystemType>::Initialize() {
      if (m_showFreeSurface) {
        m_tidal->Initialize();
        m_waveField->Initialize();
        m_freeSurfaceGridAsset->Initialize();
        m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_freeSurfaceGridAsset);
      }
    }

    template<typename OffshoreSystemType>
    void FrFreeSurface<OffshoreSystemType>::Update(double time) {
      if (m_showFreeSurface) {
        m_tidal->Update(time);
        m_waveField->Update(time);
      }
    }

    template<typename OffshoreSystemType>
    void FrFreeSurface<OffshoreSystemType>::ShowFreeSurface(bool showFreeSurface) {
      if (showFreeSurface && m_showFreeSurface != showFreeSurface) {
        std::cout << "Be careful to set new free surface grid, wave field and tidal model" << std::endl;
      }
      m_showFreeSurface = showFreeSurface;
      if (!showFreeSurface) {
        m_waveField = std::make_unique<FrNullWaveField>(this);
        m_tidal->SetNoTidal();
        m_freeSurfaceGridAsset->SetNoGrid();
      }
    }

    template<typename OffshoreSystemType>
    void FrFreeSurface<OffshoreSystemType>::StepFinalize() {
      if (m_showFreeSurface) {
        m_waveField->StepFinalize();
      }
    }

}  // end namespace frydom
