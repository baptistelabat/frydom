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


#ifndef FRYDOM_FrRadiationModelBase_H
#define FRYDOM_FrRadiationModelBase_H

#include <memory>

#include "MathUtils/Matrix66.h"

#include "frydom/core/common/FrPhysicsItem.h"


namespace frydom {


  // Forward declarations
  class FrRadiationModel;

  class FrBody;

  class FrBEMBody;


  namespace internal {

    // Forward declaration
    class FrVariablesAddedMassBase;

    // hash defition for the map with pair as a key
    struct pair_hash {
      template<class T1, class T2>
      std::size_t operator()(const std::pair<T1, T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;
      }
    };

    class FrRadiationModelBase : public FrPhysicsItemBase {

     private:

      FrRadiationModel *m_frydomRadiationModel;
      std::unordered_map<std::pair<FrBEMBody *, FrBEMBody *>, mathutils::Matrix66<double>, pair_hash> m_invGeneralizedMass;

     public:

      explicit FrRadiationModelBase(FrRadiationModel *radiationModel);

      //
      // Update
      //

      void SetupInitial() override;

      void Update(bool update_assets) override;

      void Update(double time, bool update_assets) override;

      //
      // Descriptor
      //

      void IntLoadResidual_Mv(const unsigned int off,
                              chrono::ChVectorDynamic<> &R,
                              const chrono::ChVectorDynamic<> &w,
                              const double c
      ) override;

      void IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta &v,
                           const chrono::ChVectorDynamic<> &R, const unsigned int off_L,
                           const chrono::ChVectorDynamic<> &L, const chrono::ChVectorDynamic<> &Qc) override;

      void IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta &v,
                             const unsigned int off_L, chrono::ChVectorDynamic<> &L) override;

      int GetBodyOffset(FrBody *body) const;

      void InjectVariablesToBody();

      //
      // ADDED MASS
      //

      FrRadiationModel *GetRadiationModel() const { return m_frydomRadiationModel; }

      void BuildGeneralizedMass();

      mathutils::Matrix66<double> GetInverseGeneralizedMass(FrBEMBody *BEMBody, FrBEMBody *BEMBodyMotion) const;

      mathutils::Matrix66<double> GetGeneralizedMass(FrBEMBody *BEMBody, FrBEMBody *BEMBodyMotion) const;
    };


  } // end namespace internal

}  // end namespace frydom


#endif //FRYDOM_FrRadiationModelBase_H
