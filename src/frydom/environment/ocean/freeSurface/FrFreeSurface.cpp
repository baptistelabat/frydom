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

    FrFreeSurface::~FrFreeSurface() = default;

    FrFreeSurface::FrFreeSurface(FrOcean* ocean) : m_ocean(ocean) {

        // Creating a waveField and a tidal model
        m_waveField         = std::make_unique<FrNullWaveField>(this);
        m_tidal             = std::make_unique<FrTidal>(this);
        m_freeSurfaceGridAsset    = std::make_shared<FrFreeSurfaceGridAsset>(this);

    }

    FrAtmosphere *FrFreeSurface::GetAtmosphere() const { return m_ocean->GetEnvironment()->GetAtmosphere(); }

    FrOcean *FrFreeSurface::GetOcean() const { return m_ocean; }

    FrTidal *FrFreeSurface::GetTidal() const { return m_tidal.get(); }

    FrWaveField * FrFreeSurface::GetWaveField() const { return m_waveField.get(); }

    double FrFreeSurface::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
        m_waveField->GetElevation(x,y, fc);
    }

    FrFreeSurfaceGridAsset *FrFreeSurface::GetFreeSurfaceGridAsset() const {return m_freeSurfaceGridAsset.get();}

    double FrFreeSurface::GetPosition(FRAME_CONVENTION fc) const {
        return GetPosition(0.,0.,fc);
    }

    double FrFreeSurface::GetPosition(double x, double y, FRAME_CONVENTION fc) const {
        return m_tidal->GetHeight(fc) + m_waveField->GetElevation(x, y, fc);
    }

    double FrFreeSurface::GetPosition(const Position worldPos, FRAME_CONVENTION fc) const {
        return GetPosition(worldPos[0],worldPos[1],fc);
    }

    void FrFreeSurface::GetPosition(Position& worldPos, FRAME_CONVENTION fc) const {
        worldPos[2] = GetPosition(worldPos[0],worldPos[1],fc);
    }


    void FrFreeSurface::NoWaves() {
        m_waveField = std::make_unique<FrNullWaveField>(this);
    }

    FrAiryRegularWaveField*
    FrFreeSurface::SetAiryRegularWaveField() {
        m_waveField = std::make_unique<FrAiryRegularWaveField>(this);
        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
    }

    FrAiryRegularWaveField*
    FrFreeSurface::SetAiryRegularWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirAngle, unit, fc, dc);
        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
    }


    FrAiryRegularWaveField*
    FrFreeSurface::SetAiryRegularWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirection, fc, dc);
        return waveField;
    }


    FrAiryRegularOptimWaveField*
    FrFreeSurface::SetAiryRegularOptimWaveField() {
        m_waveField = std::make_unique<FrAiryRegularOptimWaveField>(this);
        return dynamic_cast<FrAiryRegularOptimWaveField*>(m_waveField.get());
    }

    FrAiryRegularOptimWaveField*
    FrFreeSurface::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularOptimWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirAngle, unit, fc, dc);
        return waveField;
    }

    FrAiryRegularOptimWaveField*
    FrFreeSurface::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularOptimWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirection, fc, dc);
        return waveField;
    }


    FrAiryIrregularWaveField*
    FrFreeSurface::SetAiryIrregularWaveField() {
        m_waveField = std::make_unique<FrAiryIrregularWaveField>(this);
        return dynamic_cast<FrAiryIrregularWaveField*>(m_waveField.get());
    }

    FrAiryIrregularOptimWaveField*
    FrFreeSurface::SetAiryIrregularOptimWaveField() {
        m_waveField = std::make_unique<FrAiryIrregularOptimWaveField>(this);
        return dynamic_cast<FrAiryIrregularOptimWaveField*>(m_waveField.get());
    }

    void FrFreeSurface::Initialize() {
        if (m_showFreeSurface) {
            m_tidal->Initialize();
            m_waveField->Initialize();
            m_freeSurfaceGridAsset->Initialize();
            m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_freeSurfaceGridAsset);
        }
    }

    void FrFreeSurface::Update(double time) {
        if (m_showFreeSurface) {
            m_tidal->Update(time);
            m_waveField->Update(time);
        }
    }

    void FrFreeSurface::ShowFreeSurface(bool showFreeSurface) {
        if (showFreeSurface && m_showFreeSurface!=showFreeSurface) {
            std::cout<< "Be careful to set new free surface grid, wave field and tidal model"<<std::endl;
        }
        m_showFreeSurface = showFreeSurface;
        if (!showFreeSurface) {
            m_waveField = std::make_unique<FrNullWaveField>(this);
            m_tidal->SetNoTidal();
            m_freeSurfaceGridAsset->SetNoGrid();
        }
    }

    void FrFreeSurface::StepFinalize() {
        if (m_showFreeSurface) {
            m_waveField->StepFinalize();
            m_freeSurfaceGridAsset->StepFinalize();
        }
    }

}  // end namespace frydom
