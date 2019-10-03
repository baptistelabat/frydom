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


#ifndef FRYDOM_FRASSET_H
#define FRYDOM_FRASSET_H

#include "chrono/assets/ChAssetLevel.h"

#include "frydom/asset/FrAssetOwner.h"


namespace frydom {

  class FrAsset;

  namespace internal {

    struct FrAssetBase : public chrono::ChAssetLevel {

      FrAsset *m_frydomAsset;

      explicit FrAssetBase(FrAsset *asset);

//            void Update(chrono::ChPhysicsItem* updater, const chrono::ChCoordsys<>& coords) override;

    };

  }  // end namespace frydom::internal

  /**
   * \class FrAsset
   * \brief
   */
  class FrAsset {

   protected:
    std::shared_ptr<internal::FrAssetBase> m_chronoAsset;

   public:

    FrAsset();

    virtual void Initialize() = 0;

    /// Update the state of the asset, at the end of a time step
    virtual void StepFinalize() = 0;

   protected:

    std::shared_ptr<chrono::ChAsset> GetChronoAsset();

   private:

    friend void FrAssetOwner::AddAsset(std::shared_ptr<FrAsset>);
  };


}   // end namespace frydom


#endif //FRYDOM_FRASSET_H
