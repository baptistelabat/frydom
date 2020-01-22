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

#ifndef FRYDOM_FRHYDROSTATICEQUILIBRIUM_H
#define FRYDOM_FRHYDROSTATICEQUILIBRIUM_H

#include <memory>
#include "MathUtils/Constants.h"
#include "frydom/core/common/FrFrame.h"
//#include <frydom/mesh/FrHydrostaticsProperties.h>

namespace frydom {

  // Forward declarations
  class FrBody;

  class FrHydroMesh;

  class FrInertiaTensor;

  /**
   * \class FrHydroStaticEquilibrium
   * \brief Class for solving the hydrostatic equilibrium of a body, subject to an inertia tensor.
   * The hydrostatic equilibrium is obtained using a Newton-Raphson solver, with the jacobian being the stiffness matrix
   * computed after each body position and attitude modification.
   */
  class FrHydroStaticEquilibrium {

   public:

    /// Constructor for the FrHydrostaticEquilibrium, applied to a body, using a mesh, given in the meshFile, which can
    /// have a reference frame different than the body reference frame. The transformation frame from the mesh reference
    /// frame to the body frame is given by meshOffset.
    /// \param body body for which is solved the hydrostatic equilibrium
    /// \param meshFile name of the file containing the mesh
    /// \param meshOffset mesh frame offset, relatively to the body reference frame
    FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body,
                             const std::string &meshFile,
                             FrFrame meshOffset);

    /// FrHydrostaticEquilibrium destructor to ensure that the HydroMesh created for the solving is removed to the system.
    ~FrHydroStaticEquilibrium();

    /// Solve the hydrostatic equilibrium of a body, modeled by a mesh, subject to an inertia tensor.
    /// \param tensor inertia tensor for which the hydrostatic equilibrium is solved
    /// \return true if the solver reaches a convergence, even an unstable configuration.
    bool Solve(const FrInertiaTensor &tensor);

    /// Get the hydroMesh containing the clipped mesh, resulting from the solving
    FrHydroMesh *GetHydroMesh() const;

    /// Get the hydrostatic report, containing all hydrostatic information of the clipped mesh, resulting from the solver
    /// \param COGPosInBody COG position given in the body reference frame
    /// \param refPosInBody expression point for the stiffness matrix and other hydrostatic quantities
    /// \param fc frame convention (NED/NWU) for thez two positions
    /// \return report
    std::string GetReport(const Position &COGPosInBody, const Position &refPosInBody, FRAME_CONVENTION fc) const;

    /// Set the maximum number of iterations for the Newton-Raphson solver
    /// \param max_iterations maximum iterations
    void SetMaxIterations(unsigned int max_iterations);

    /// Get the maximum number of iterations for the Newton-Raphson solver
    /// \return maximum iterations
    unsigned int GetMaxIterations() const;

    /// Set the relative tolerance for stopping the Newton-Raphson solver
    /// \param tolerance relative tolerance
    void SetRelativeTolerance(double tolerance);

    /// Get the relative tolerance for stopping the Newton-Raphson solver
    /// \return relative tolerance
    double GetRelativeTOlerance() const;

    /// Set the linear part of the relaxation applied to the solution of the Newton-Raphson, in meters. The evolution of
    /// the linear part of the solution, between two iterations, cannot exceed the relaxation given.
    /// \param relaxation linear part of the relation, in meters
    void SetLinearRelaxation(double relaxation);

    /// Set the angular part of the relaxation applied to the solution of the Newton-Raphson, in radians. The evolution of
    /// the angular part of the solution, between two iterations, cannot exceed the relaxation given.
    /// \param relaxation linear part of the relation, in radians
    void SetAngularRelaxation(double relaxation);

    /// Set the relaxation vector applied to the solution of the Newton-Raphson. The first component is in meters, the
    /// two others are in radians.
    /// \param relaxation relaxation vector
    void SetRelaxation(Vector3d<double> &relaxation);

    /// Get the relaxation vector applied to the solution of the Newton-Raphson. The first component is in meters, the
    /// two others are in radians.
    /// \return relaxation vector
    Vector3d<double> GetRelaxation() const;

    FrFrame GetEquilibriumFrame() const;

   private:
    std::shared_ptr<FrBody> m_body;            ///< body for which the hydrostatic equilibrium is solved
    std::shared_ptr<FrHydroMesh> m_hydroMesh;  ///< hydroMesh containing the original body mesh and the clipping process

    // Newton-Raphson quantities
    Vector3d<double> m_residual;                ///< residual of the Newton-Raphson solver
    Vector3d<double> m_solution;                ///< solution of the Newton-Raphson solver

    unsigned int m_iteration = 0;               ///< number of Newton-Raphson iterations
    unsigned int m_iterations_max = 100;        ///< number of maximum Newton-Raphson iterations
    double m_relative_tolerance = 1e-2;         ///< relative tolerance for stopping the Newton-Raphson solver

    Vector3d<double> m_relax;                   ///< relaxation vector, to ensure that the evolution of the solution is
    ///< not to large

  };

  /// maker of the hydrostatic equilibrium solver, for a body
  /// \param body
  /// \param meshFile
  /// \param meshOffset
  /// \param tensor
  /// \return
  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset,
                                const FrInertiaTensor &tensor);


} //end namespace frydom


#endif //FRYDOM_FRHYDROSTATICEQUILIBRIUM_H
