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
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"


namespace frydom {

    // FrSeabed descriptions

    template<typename OffshoreSystemType>
    FrSeabed<OffshoreSystemType>::FrSeabed(FrOcean<OffshoreSystemType> *ocean) :m_ocean(ocean) {
    }

    template<typename OffshoreSystemType>
    FrOcean<OffshoreSystemType> *FrSeabed<OffshoreSystemType>::GetOcean() const { return m_ocean; }

    template<typename OffshoreSystemType>
    bool FrSeabed<OffshoreSystemType>::GetInfiniteDepth() { return m_infiniteDepth; }

    //------------------------------------------------------------------------------------------------------------------
    // FrNullSeabed descriptions
    template<typename OffshoreSystemType>
    FrSeabedGridAsset<OffshoreSystemType> *FrNullSeabed<OffshoreSystemType>::GetSeabedGridAsset() {
      try { throw FrException("a null seabed cannot return a seabed asset."); }
      catch (FrException &e) {
        std::cout << e.what() << std::endl;
        exit(EXIT_FAILURE);
      }
    }

    template<typename OffshoreSystemType>
    FrNullSeabed<OffshoreSystemType>::FrNullSeabed(FrOcean<OffshoreSystemType> *ocean) : FrSeabed<OffshoreSystemType>(
        ocean) { this->m_infiniteDepth = true; }

    template<typename OffshoreSystemType>
    void FrNullSeabed<OffshoreSystemType>::SetBathymetry(double bathymetry, FRAME_CONVENTION fc) {
      try { throw FrException("a null seabed cannot return a bathymetry."); }
      catch (FrException &e) {
        std::cout << e.what() << std::endl;
        exit(EXIT_FAILURE);
      }
    }

    template<typename OffshoreSystemType>
    const double FrNullSeabed<OffshoreSystemType>::GetBathymetry(FRAME_CONVENTION fc) const {
      try { throw FrException("a null seabed cannot return a bathymetry."); }
      catch (FrException &e) {
        std::cout << e.what() << std::endl;
        exit(EXIT_FAILURE);
      }
    }

    template<typename OffshoreSystemType>
    const double FrNullSeabed<OffshoreSystemType>::GetBathymetry(double x, double y, FRAME_CONVENTION fc) const {
      try { throw FrException("a null seabed cannot return a bathymetry."); }
      catch (FrException &e) {
        std::cout << e.what() << std::endl;
        exit(EXIT_FAILURE);
      }
    }

    template<typename OffshoreSystemType>
    void FrNullSeabed<OffshoreSystemType>::Update(double time) {

    }

    template<typename OffshoreSystemType>
    void FrNullSeabed<OffshoreSystemType>::Initialize() {

    }

    template<typename OffshoreSystemType>
    void FrNullSeabed<OffshoreSystemType>::StepFinalize() {

    }

    //------------------------------------------------------------------------------------------------------------------
    // FrMeanSeabed descriptions
    template<typename OffshoreSystemType>
    FrMeanSeabed<OffshoreSystemType>::FrMeanSeabed(FrOcean<OffshoreSystemType> *ocean) : FrSeabed<OffshoreSystemType>(
        ocean) {
      m_SeabedGridAsset = std::make_shared<FrSeabedGridAsset>(this);
    }

    template<typename OffshoreSystemType>
    void FrMeanSeabed<OffshoreSystemType>::SetBathymetry(double bathymetry, FRAME_CONVENTION fc) {
      assert(m_showSeabed);
      if (IsNED(fc)) { bathymetry = -bathymetry; };
      m_bathymetry = bathymetry;
    }

    template<typename OffshoreSystemType>
    const double FrMeanSeabed<OffshoreSystemType>::GetBathymetry(FRAME_CONVENTION fc) const {
      assert(m_showSeabed);
      double bathy = m_bathymetry;
      if (IsNED(fc)) { bathy = -bathy; }
      return bathy;
    }

    template<typename OffshoreSystemType>
    const double FrMeanSeabed<OffshoreSystemType>::GetBathymetry(double x, double y, FRAME_CONVENTION fc) const {
      assert(m_showSeabed);
      double bathy = m_bathymetry;
      if (IsNED(fc)) { bathy = -bathy; }
      return bathy;
    }

    template<typename OffshoreSystemType>
    void FrMeanSeabed<OffshoreSystemType>::Update(double time) {}

    template<typename OffshoreSystemType>
    void FrMeanSeabed<OffshoreSystemType>::Initialize() {
      if (m_showSeabed) {
        m_SeabedGridAsset->Initialize();
        this->m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_SeabedGridAsset);
      }
    }

    template<typename OffshoreSystemType>
    void FrMeanSeabed<OffshoreSystemType>::StepFinalize() {

    }

    template<typename OffshoreSystemType>
    FrSeabedGridAsset<OffshoreSystemType> *
    FrMeanSeabed<OffshoreSystemType>::GetSeabedGridAsset() { return m_SeabedGridAsset.get(); }


}  // end namespace frydom
