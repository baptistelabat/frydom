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

#include <iostream>

#include "FrSeabed.h"

#include "frydom/asset/FrSeabedGridAsset.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"


namespace frydom {

    // FrSeabed descriptions

    FrSeabed::FrSeabed(FrOcean *ocean) :m_ocean(ocean){
    }

    FrOcean *FrSeabed::GetOcean() const {return m_ocean;}

    bool FrSeabed::GetInfiniteDepth() { return m_infiniteDepth;}

    //------------------------------------------------------------------------------------------------------------------
    // FrNullSeabed descriptions

    FrSeabedGridAsset *FrNullSeabed::GetSeabedGridAsset() {
        try {throw FrException("a null seabed cannot return a seabed asset.");}
        catch(FrException& e) {std::cout<<e.what()<<std::endl; exit(EXIT_FAILURE);}
    }

    FrNullSeabed::FrNullSeabed(FrOcean *ocean) :FrSeabed(ocean) {m_infiniteDepth = true;}

    void FrNullSeabed::SetBathymetry(double bathymetry, FRAME_CONVENTION fc) {
        try {throw FrException("a null seabed cannot return a bathymetry.");}
        catch(FrException& e) {std::cout<<e.what()<<std::endl; exit(EXIT_FAILURE);}
    }

    const double FrNullSeabed::GetBathymetry(FRAME_CONVENTION fc) const {
        try {throw FrException("a null seabed cannot return a bathymetry.");}
        catch(FrException& e) {std::cout<<e.what()<<std::endl; exit(EXIT_FAILURE);}
    }

    const double FrNullSeabed::GetBathymetry(double x, double y, FRAME_CONVENTION fc) const {
        try {throw FrException("a null seabed cannot return a bathymetry.");}
        catch(FrException& e) {std::cout<<e.what()<<std::endl; exit(EXIT_FAILURE);}
    }

    void FrNullSeabed::Update(double time) {

    }

    void FrNullSeabed::Initialize() {

    }

    void FrNullSeabed::StepFinalize() {

    }

    //------------------------------------------------------------------------------------------------------------------
    // FrMeanSeabed descriptions

    FrMeanSeabed::FrMeanSeabed(FrOcean *ocean) :FrSeabed(ocean){
        m_SeabedGridAsset = std::make_shared<FrSeabedGridAsset>(this);
    }

    void FrMeanSeabed::SetBathymetry(double bathymetry, FRAME_CONVENTION fc) {
        assert(m_showSeabed);
        if (IsNED(fc)) {bathymetry = -bathymetry;};
        m_bathymetry = bathymetry;
    }

    const double FrMeanSeabed::GetBathymetry(FRAME_CONVENTION fc) const {
        assert(m_showSeabed);
        double bathy = m_bathymetry;
        if (IsNED(fc)) {bathy = -bathy;}
        return bathy;
    }

    const double FrMeanSeabed::GetBathymetry(double x, double y, FRAME_CONVENTION fc) const {
        assert(m_showSeabed);
        double bathy = m_bathymetry;
        if (IsNED(fc)) {bathy = -bathy;}
        return bathy;
    }

    void FrMeanSeabed::Update(double time) {}

    void FrMeanSeabed::Initialize() {
        if (m_showSeabed) {
            m_SeabedGridAsset->Initialize();
            m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_SeabedGridAsset);
        }
    }

    void FrMeanSeabed::StepFinalize() {

    }

    FrSeabedGridAsset *FrMeanSeabed::GetSeabedGridAsset() {return m_SeabedGridAsset.get();}


}  // end namespace frydom
