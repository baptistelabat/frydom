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


#ifndef FRYDOM_FRLINEARDIFFRACTIONFORCE_H
#define FRYDOM_FRLINEARDIFFRACTIONFORCE_H

#include <memory>
#include <vector>

#include "MathUtils/Matrix66.h"
#include "frydom/core/force/FrForce.h"
#include "FrLinearExcitationForceBase.h"

namespace frydom {

    // Forward declaration
    template<typename OffshoreSystemType>
    class FrHydroDB;

    template<typename OffshoreSystemType>
    class FrBody;

    template<typename OffshoreSystemType>
    class FrEquilibriumFrame;

    /**
     * \class FrLinearDiffractionForce
     * \brief Class for computing the linear diffraction loads.
     */
    template<typename OffshoreSystemType>
    class FrLinearDiffractionForce : public FrLinearExcitationForceBase<OffshoreSystemType> {

     public:

      /// Constructor.
      explicit FrLinearDiffractionForce(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB)
          : FrLinearExcitationForceBase<OffshoreSystemType>(HDB) {};

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "LinearDiffractionForce"; }

      void Initialize() override;

      Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

      Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

     private:

      /// Compute the linear diffraction force
      /// \param time Current time of the simulation from beginning, in seconds
      void Compute(double time) override;

    };

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearDiffractionForce<OffshoreSystemType>>
    make_linear_diffraction_force(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB,
                                  std::shared_ptr<FrBody<OffshoreSystemType>> body);


}  // end namespace frydom

#endif //FRYDOM_FRLINEARDIFFRACTIONFORCE_H
