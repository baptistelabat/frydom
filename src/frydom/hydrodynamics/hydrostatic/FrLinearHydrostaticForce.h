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


#ifndef FRYDOM_FRLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRLINEARHYDROSTATICFORCE_H


#include <memory>
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/force/FrForce.h"
#include "FrLinearHydrostaticStiffnessMatrix.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"

namespace frydom {

    // Forward declarations
    template<typename OffshoreSystemType>
    class FrEquilibriumFrame;
//    class FrHydroDB;

    /// This class defines the linear hydrostatic restoring force applied to a hydrodynamic body.
    /// The force takes into account the position of the body (at COG) regards to the equilibrium frame.
    /// For rotation, cardan angle are considered.
    /// It is supposed that the equilibrium frame has the z-axis pointing upwards and its
    /// position equals the position of the COG of the body at equilibrium

    /**
     * \class FrLinearHydrostaticForce
     * \brief Class for computing linear hydrostatic loads.
     */
    template<typename OffshoreSystemType>
    class FrLinearHydrostaticForce : public FrForce<OffshoreSystemType> {

     private:
//        std::shared_ptr<FrHydroDB> m_HDB;
      FrLinearHydrostaticStiffnessMatrix m_stiffnessMatrix;      ///< Hydrostatic stiffness matrix
      //TODO: passed the raw to shared ptr, need some modif in the mapper.
      FrEquilibriumFrame<OffshoreSystemType> *m_equilibriumFrame;    ///< Equilibrium frame of the body to which the force is applied

      /// Boolean to know if the hydrostatic matrix is obtained from the HDB5 file (true) or not (false).
      bool HydrostaticsMatrixHDB5 = true;

     public:

      /// Constructor.
      explicit FrLinearHydrostaticForce(const std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> &eqFrame);

      explicit FrLinearHydrostaticForce(FrEquilibriumFrame<OffshoreSystemType> *eqFrame);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "LinearHydrostaticForce"; }

      /// Return true if the force is included in the static analysis
      bool IncludedInStaticAnalysis() const override { return true; }

      /// Get the hydrostatic stiffness matrix of the hydrostatic force
      /// \return Hydrostatic stiffness matrix
      FrLinearHydrostaticStiffnessMatrix GetStiffnessMatrix() const;

      /// This function sets the hydrostatic stiffness matrix.
      void SetStiffnessMatrix(FrLinearHydrostaticStiffnessMatrix HydrostaticMatrix);

      /// This function sets the hydrostatic stiffness matrix.
      void SetStiffnessMatrix(mathutils::MatrixMN<double> HydrostaticMatrix);

      /// Intialize the linear hydrostatic force model
      void Initialize() override;

     private:

      /// Compute the linear hydrostatic force
      /// \param time Current time of the simulation from beginning
      void Compute(double time) override;

    };

    /// This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a
    /// hydrostatic sitffness matrix computed by FrMesh.
    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearHydrostaticForce<OffshoreSystemType>>
    make_linear_hydrostatic_force(const std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> &eqFrame,
                                  const std::shared_ptr<FrBody<OffshoreSystemType>> &body);

    /// This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a
    /// hydrostatic stiffness matrix given by the hdb.
    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearHydrostaticForce<OffshoreSystemType>>
    make_linear_hydrostatic_force(const std::shared_ptr<FrHydroDB<OffshoreSystemType>> &HDB,
                                  const std::shared_ptr<FrBody<OffshoreSystemType>> &body);

    /// This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a
    /// hydrostatic sitffness matrix computed by FrMesh.
    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearHydrostaticForce<OffshoreSystemType>>
    make_linear_hydrostatic_force(const std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> &eqFrame,
                                  const std::shared_ptr<FrBody<OffshoreSystemType>> &body,
                                  const std::string &meshFile, FrFrame meshOffset);

}  // end namespace frydom


#endif //FRYDOM_FRLINEARHYDROSTATICFORCE_H
