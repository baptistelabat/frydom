//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRLINEARHYDROSTATICFORCE_H

// FIXME: attention, on a d'autres fichiers FrHydrostaticForce.h et .cpp
#include "chrono/physics/ChBody.h"
#include <frydom/core/force/FrForce.h>
#include <frydom/core/junk/FrHydroBody.h>
#include "FrLinearHydrostaticStiffnessMatrix.h"

// <<<<<<<<<<<<<<<<<<< Additional include from refactoring

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"

// <<<<<<<<<<<<<<<<<<


// FIXME: bien travailler sur le bon placement de la force hydrostatique lineaire !!!!

// TODO: Attention, il faut fonctionner avec une difference de position

namespace frydom {

    class FrLinearHydrostaticForce : public FrForce {

    private:
        FrLinearHydrostaticStiffnessMatrix m_stiffnessMatrix;

        // ##CC fix log
        double m_delta_z;
        double m_body_z;
        double m_eqframe_z;
        // ##CC

    public:

        FrLinearHydrostaticForce();

        FrLinearHydrostaticStiffnessMatrix* GetStiffnessMatrix();


        void UpdateState() override;

        // ##CC Fix pour le log
        void InitializeLogs() override;
        // ##CC

        void SetLogPrefix(std::string prefix_name) override;

    };




















    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    // forward declaration

    class FrEquilibriumFrame_;

    /// This class defines the linear hydrostatic restoring force applied to a hydrodynamic body.
    /// The force take into account the position of the body (at COG) regards to the equilibrium frame.
    /// For rotation, cardan angle are considered.
    /// It is supposed that the equilibrium frame has the z-axis pointing upwards and its
    /// position equals the position of the COG of the body at equilibrium

    class FrLinearHydrostaticForce_ : public FrForce_ {

    private:
        std::shared_ptr<FrHydroDB_> m_HDB;
        FrLinearHydrostaticStiffnessMatrix_ m_stiffnessMatrix;      ///< Hydrostatic stiffness matrix
        //TODO: passed the raw to shared ptr, need some modif in the mapper.
        FrEquilibriumFrame_* m_equilibriumFrame;    ///< Equilibrium frame of the body to which the force is applied

    public:
        FrLinearHydrostaticForce_(std::shared_ptr<FrHydroDB_> HDB) : m_HDB(HDB) { }

        /// Get the stiffness matrix of the hydrostatic force
        /// \return Hydrostatic stiffness matrix
        FrLinearHydrostaticStiffnessMatrix_* GetStiffnessMatrix() { return &m_stiffnessMatrix; }

        /// Update linear hydrostatic force
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Intialize the hydrostatic force model
        void Initialize() override;

        /// Methods to be applied at the end of each time steps
        void StepFinalize() override { }
    };


    std::shared_ptr<FrLinearHydrostaticForce_>
    make_linear_hydrostatic_force(std::shared_ptr<FrHydroDB_> HDB, std::shared_ptr<FrBody_> body);


}  // end namespace frydom


#endif //FRYDOM_FRLINEARHYDROSTATICFORCE_H
