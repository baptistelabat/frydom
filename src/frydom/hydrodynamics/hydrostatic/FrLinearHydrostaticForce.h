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
#include "frydom/core/force/FrForce.h"
#include "FrLinearHydrostaticStiffnessMatrix.h"


namespace frydom {

    // Forward declarations
    class FrEquilibriumFrame;
    class FrHydroDB;

    /// This class defines the linear hydrostatic restoring force applied to a hydrodynamic body.
    /// The force take into account the position of the body (at COG) regards to the equilibrium frame.
    /// For rotation, cardan angle are considered.
    /// It is supposed that the equilibrium frame has the z-axis pointing upwards and its
    /// position equals the position of the COG of the body at equilibrium

    /**
     * \class FrLinearHydrostaticForce
     * \brief Class for computing linear hydrostatic loads.
     */
    class FrLinearHydrostaticForce : public FrForce {

    private:
        std::shared_ptr<FrHydroDB> m_HDB;
        FrLinearHydrostaticStiffnessMatrix m_stiffnessMatrix;      ///< Hydrostatic stiffness matrix
        //TODO: passed the raw to shared ptr, need some modif in the mapper.
        FrEquilibriumFrame* m_equilibriumFrame;    ///< Equilibrium frame of the body to which the force is applied

    public:

        /// Constructor.
        explicit FrLinearHydrostaticForce(std::shared_ptr<FrHydroDB> HDB) : m_HDB(HDB) { }

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearHydrostaticForce"; }

        /// Return true if the force is included in the static analysis
        bool IncludedInStaticAnalysis() const override {return true;}

        /// Get the stiffness matrix of the hydrostatic force
        /// \return Hydrostatic stiffness matrix
        FrLinearHydrostaticStiffnessMatrix* GetStiffnessMatrix() { return &m_stiffnessMatrix; }

        /// Intialize the hydrostatic force model
        void Initialize() override;

        /// Methods to be applied at the end of each time steps
        void StepFinalize() override;

    private:

        /// Compute the linear hydrostatic force
        /// \param time Current time of the simulation from beginning
        void Compute(double time) override;

    };

    /// This subroutine reads the modes of a body.
    std::shared_ptr<FrLinearHydrostaticForce>
    make_linear_hydrostatic_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body);


}  // end namespace frydom


#endif //FRYDOM_FRLINEARHYDROSTATICFORCE_H
