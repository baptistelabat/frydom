//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRLINEARHYDROSTATICFORCE_H

// FIXME: attention, on a d'autres fichiers FrHydrostaticForce.h et .cpp
#include "chrono/physics/ChBody.h"
#include <frydom/core/FrForce.h>
#include <frydom/core/FrHydroBody.h>
#include "FrLinearHydrostaticStiffnessMatrix.h"

/// <<<<<<<<<<<<<<<<<<< Additional include from refactoring


/// <<<<<<<<<<<<<<<<<<


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

    class FrLinearHydrostaticForce_ : public FrForce_ {

    private:
        FrLinearHydrostaticStiffnessMatrix_ m_stiffnessMatrix;
        std::shared_ptr<FrEquilibriumFrame_> m_equilibriumFrame;

    public:
        /// Constructor of a linear hydrostatic force with equilibrium frame
        /// \param equilibriumFrame Equilibrium frame
        FrLinearHydrostaticForce_(std::shared_ptr<FrEquilibriumFrame_> equilibriumFrame)
                : FrForce_(), m_equilibriumFrame(equilibriumFrame) { }

        ///
        /// \param stiffnessMatrix
        void SetStiffnessMatrix(FrLinearHydrostaticStiffnessMatrix_ stiffnessMatrix) { m_stiffnessMatrix = stiffnessMatrix; }

        FrLinearHydrostaticStiffnessMatrix_* GetStiffnessMatrix() { return &m_stiffnessMatrix; }

        void Update(double time) override;

        void Initialize() override { }

        void StepFinalize() override { }
    };

}  // end namespace frydom


#endif //FRYDOM_FRLINEARHYDROSTATICFORCE_H
