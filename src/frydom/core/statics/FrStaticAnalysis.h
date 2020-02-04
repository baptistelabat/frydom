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

#ifndef FRYDOM_FRSTATICANALYSIS_H
#define FRYDOM_FRSTATICANALYSIS_H


#include <map>
#include "frydom/core/common/FrObject.h"


namespace frydom {

  // Forward declarations
  class FrOffshoreSystem;

  class FrStaticAnalysis : public FrObject {

   public:

    /// enum for the relaxation methods
    enum RELAXTYPE {
      NORELAX,                ///< no relaxation
      VELOCITY,               ///< velocities are set to null
      ACCELERATION,           ///< accelelrations are set to null
      VELOCITYANDACCELERATION ///< velocities and accelerations are set to null
    };

   private:
    FrOffshoreSystem *m_system;     ///< Pointer to the offshore system, containing this structure

    int m_nIterations = 10;         ///< Number of iterations for the static procedure
    /// each iteration contains m_nSteps steps; after each iteration a relaxation is applied
    int m_nSteps = 100;             ///< Relaxation is applied every m_nSteps steps

    double m_tolerance = 1E-3;      ///< tolerance value, to check if the static equilibrium is reached

    RELAXTYPE m_relax = VELOCITY;   ///< relaxation method used

    std::map<FrObject *, std::pair<bool, bool>> m_map;    ///< Map to keep the activity and log activity of
    ///< elements before the static analysis

    // Temporary stored variables
    double m_undoTime = 0.;         ///< simulation time before the static analysis, to be stored

    double m_x0, m_y0, m_x1, m_y1;    ///< time ramp parameters, as set in the system before the static analysis
//        FrCosRampFunction* m_ramp;      ///< time ramp, as set in the system before the static analysis

    double c_residual;
    int c_iter;

   public:

    explicit FrStaticAnalysis(FrOffshoreSystem *system);;

    FrOffshoreSystem *GetSystem();

    /// Set the number of steps between two relaxations, during static iterations
    /// \param nSteps number of steps between two relaxations
    void SetNbSteps(int nSteps);

    /// Get the number of steps between two relaxations, during static iterations
    /// \return number of steps between two relaxations
    int GetNbSteps() const;

    /// Set the maximum number of iterative steps to find the static equilibrium,
    /// \param nIter maximum number of iterative steps
    void SetNbIteration(int nIter);

    /// Get the maximum number of iterative steps to find the static equilibrium,
    /// \return maximum number of iterative steps
    int GetNbIteration() const;

    /// Set the relaxation procedure in the static solving :
    /// none, only velocities set to null, only accelerations set to null, velocities and accelerations set to null
    /// \param relax relaxation procedure (NONE, VELOCITY, ACCELERATION, VELOCITYANDACCELERATION)
    void SetRelaxation(RELAXTYPE relax);

    /// Get the relaxation procedure in the static solving :
    /// none, only velocities set to null, only accelerations set to null, velocities and accelerations set to null
    /// \return relaxation procedure (NONE, VELOCITY, ACCELERATION, VELOCITYANDACCELERATION)
    RELAXTYPE GetRelaxation() const;

    /// Set the tolerance criteria to stop the static solving
    /// \param tol tolerance criteria to stop the static solving
    void SetTolerance(double tol);

    /// Get the tolerance criteria to stop the static solving
    /// \return tolerance criteria to stop the static solving
    double GetTolerance() const;

    /// Solve the static equilibrium using a dynamic simulation with relaxations (velocities and/or accelerations of
    /// bodies set to null) every nSteps steps. The maximum number of relaxation is defined by nIter. The solving
    /// stops if nIter or the static tolerance is reached.
    bool SolveStatic();

   private:

    /// Initialize the static by deactivating the bodies/links/physics items not included in the static analysis
    void Initialize() override;

    /// Finalize the static analysis by creating a report and setting the elements to their previous state
    void StepFinalize() override;

  };


} //end namespace frydom

#endif //FRYDOM_FRSTATICANALYSIS_H
