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


#include "MathUtils/MathUtils.h"

#include "FrFreeSurface.h"
#include "frydom/core/FrOffshoreSystem.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "frydom/asset/FrFreeSurfaceGridAsset.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveProbe.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryRegularWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryRegularOptimWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryIrregularWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryIrregularOptimWaveField.h"


#include "frydom/core/body/FrBody.h"


namespace frydom {

    FrFreeSurface_::~FrFreeSurface_() = default;

    FrFreeSurface_::FrFreeSurface_(FrOcean_* ocean) : m_ocean(ocean) {

        // Creating a waveField and a tidal model
        m_waveField         = std::make_unique<FrNullWaveField_>(this);
        m_tidal             = std::make_unique<FrTidal_>(this);
        m_freeSurfaceGridAsset    = std::make_shared<FrFreeSurfaceGridAsset>(this);

    }

    FrAtmosphere_ *FrFreeSurface_::GetAtmosphere() const { return m_ocean->GetEnvironment()->GetAtmosphere(); }

    FrOcean_ *FrFreeSurface_::GetOcean() const { return m_ocean; }

    FrTidal_ *FrFreeSurface_::GetTidal() const { return m_tidal.get(); }

    FrWaveField_ * FrFreeSurface_::GetWaveField() const { return m_waveField.get(); }

    double FrFreeSurface_::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
        m_waveField->GetElevation(x,y, fc);
    }

    FrFreeSurfaceGridAsset *FrFreeSurface_::GetFreeSurfaceGridAsset() const {return m_freeSurfaceGridAsset.get();}

    double FrFreeSurface_::GetPosition(FRAME_CONVENTION fc) const {
        return GetPosition(0.,0.,fc);
    }

    double FrFreeSurface_::GetPosition(double x, double y, FRAME_CONVENTION fc) const {
        return m_tidal->GetHeight(fc) + m_waveField->GetElevation(x, y, fc);
    }

    double FrFreeSurface_::GetPosition(const Position worldPos, FRAME_CONVENTION fc) const {
        return GetPosition(worldPos[0],worldPos[1],fc);
    }

    void FrFreeSurface_::GetPosition(Position& worldPos, FRAME_CONVENTION fc) const {
        worldPos[2] = GetPosition(worldPos[0],worldPos[1],fc);
    }


    void FrFreeSurface_::NoWaves() {
        m_waveField = std::make_unique<FrNullWaveField_>(this);
    }

    FrAiryRegularWaveField*
    FrFreeSurface_::SetAiryRegularWaveField() {
        m_waveField = std::make_unique<FrAiryRegularWaveField>(this);
        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
    }

    FrAiryRegularWaveField*
    FrFreeSurface_::SetAiryRegularWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirAngle, unit, fc, dc);
        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
    }


    FrAiryRegularWaveField*
    FrFreeSurface_::SetAiryRegularWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirection, fc, dc);
        return waveField;
    }


    FrAiryRegularOptimWaveField*
    FrFreeSurface_::SetAiryRegularOptimWaveField() {
        m_waveField = std::make_unique<FrAiryRegularOptimWaveField>(this);
        return dynamic_cast<FrAiryRegularOptimWaveField*>(m_waveField.get());
    }

    FrAiryRegularOptimWaveField*
    FrFreeSurface_::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularOptimWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirAngle, unit, fc, dc);
        return waveField;
    }

    FrAiryRegularOptimWaveField*
    FrFreeSurface_::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularOptimWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirection, fc, dc);
        return waveField;
    }


    FrAiryIrregularWaveField*
    FrFreeSurface_::SetAiryIrregularWaveField() {
        m_waveField = std::make_unique<FrAiryIrregularWaveField>(this);
        return dynamic_cast<FrAiryIrregularWaveField*>(m_waveField.get());
    }

    FrAiryIrregularOptimWaveField*
    FrFreeSurface_::SetAiryIrregularOptimWaveField() {
        m_waveField = std::make_unique<FrAiryIrregularOptimWaveField>(this);
        return dynamic_cast<FrAiryIrregularOptimWaveField*>(m_waveField.get());
    }

    void FrFreeSurface_::Initialize() {
        if (m_showFreeSurface) {
            m_tidal->Initialize();
            m_waveField->Initialize();
            m_freeSurfaceGridAsset->Initialize();
            m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_freeSurfaceGridAsset);
        }
    }

    void FrFreeSurface_::Update(double time) {
        if (m_showFreeSurface) {
            m_tidal->Update(time);
            m_waveField->Update(time);
        }
    }

    void FrFreeSurface_::ShowFreeSurface(bool showFreeSurface) {
        if (showFreeSurface && m_showFreeSurface!=showFreeSurface) {
            std::cout<< "Be careful to set new free surface grid, wave field and tidal model"<<std::endl;
        }
        m_showFreeSurface = showFreeSurface;
        if (!showFreeSurface) {
            m_waveField = std::make_unique<FrNullWaveField_>(this);
            m_tidal->SetNoTidal();
            m_freeSurfaceGridAsset->SetNoGrid();
        }
    }

    void FrFreeSurface_::StepFinalize() {
        if (m_showFreeSurface) {
            m_waveField->StepFinalize();
            m_freeSurfaceGridAsset->StepFinalize();
        }
    }

}  // end namespace frydom
