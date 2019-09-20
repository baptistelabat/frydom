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


#ifndef FRYDOM_FRNONLINEARFROUDEKRYLOVFORCE_H
#define FRYDOM_FRNONLINEARFROUDEKRYLOVFORCE_H

#include <memory>
#include <vector>

#include "MathUtils/Matrix66.h"

#include "frydom/core/force/FrForce.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/mesh/FrHydroMesh.h"
#include "FrLinearExcitationForceBase.h"

namespace frydom {

    // Forward declarations.
    template<typename OffshoreSystemType>
    class FrHydroDB;

    template<typename OffshoreSystemType>
    class FrBody;

    template<typename OffshoreSystemType>
    class FrEquilibriumFrame;

    /**
     * \class FrNonLinearExcitationForce
     * \brief Class for computing the nonlinear excitation loads (nonlinear FK, linear diffraction).
     */
    template<typename OffshoreSystemType>
    class FrNonLinearFroudeKrylovForce : public FrForce<OffshoreSystemType> {

     private:

      std::shared_ptr<FrHydroMesh<OffshoreSystemType>> m_hydroMesh;   ///< clipped mesh container

     public:

      explicit FrNonLinearFroudeKrylovForce(const std::shared_ptr<FrHydroMesh<OffshoreSystemType>> &HydroMesh);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "NonLinearFroudeKrylovForce"; }

     private:

      void Compute(double time) override;

    };

    /// This function creates a (fully or weakly) nonlinear Froude-Krylov force object.
    template<typename OffshoreSystemType>
    std::shared_ptr<FrNonLinearFroudeKrylovForce<OffshoreSystemType>>
    make_nonlinear_froude_krylov_force(std::shared_ptr<FrBody<OffshoreSystemType>> body,
                                       std::shared_ptr<FrHydroMesh<OffshoreSystemType>> HydroMesh);

}  // end namespace frydom

#endif //FRYDOM_FRNONLINEARFROUDEKRYLOVFORCE_H
