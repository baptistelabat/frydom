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
#include "FrLinearHydrostaticStiffnessMatrix.h"

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
    /// \param mass mass for which the displacement/equilibrium is solved
    /// \param COGPosInBody COG position, in body reference frame, for which the equilibrium is solved
    /// \param fc frame convention (NED/NWU)
    FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body,
                             const std::string &meshFile,
                             FrFrame meshOffset,
                             double mass,
                             const Position& COGPosInBody, FRAME_CONVENTION fc);

    /// FrHydrostaticEquilibrium destructor to ensure that the HydroMesh created for the solving is removed to the system.
    ~FrHydroStaticEquilibrium();

    /// Solve the displacement, according to the mass specified
    /// \return true of the solver reaches a convergence.
    bool SolveDisplacement();

    /// Solve the hydrostatic equilibrium of a body, modeled by a mesh, according to the mass and COG position specified.
    /// \return true if the solver reaches a convergence, even an unstable configuration.
    bool SolveEquilibrium();

    /// Get the hydroMesh containing the clipped mesh, resulting from the solving
    FrHydroMesh *GetHydroMesh() const;

    /// Get the hydrostatic report, containing all hydrostatic information of the clipped mesh, resulting from the solver
    /// \return report
    std::string GetReport() const;

    /// Get the hydrostatic report, containing all hydrostatic information of the clipped mesh, resulting from the solver
    /// \param reductionPoint reduction point for the stiffness matrix and other hydrostatic quantities, expressed in
    /// the world reference frame
    /// \param fc frame convention (NED/NWU)
    /// \return report
    std::string GetReport(const Position &reductionPoint, FRAME_CONVENTION fc) const;

    /// Get the hydrostatic stiffness matrix, at the specified COG position.
    FrLinearHydrostaticStiffnessMatrix GetHydrostaticMatrix() const;

    /// Get the hydrostatic stiffness matrix, at the specified reduction point.
    FrLinearHydrostaticStiffnessMatrix GetHydrostaticMatrix(const Position& reductionPoint, FRAME_CONVENTION fc) const;

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
    double GetRelativeTolerance() const;

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

   private:
    std::shared_ptr<FrBody> m_body;            ///< body for which the hydrostatic equilibrium is solved
    std::shared_ptr<FrHydroMesh> m_hydroMesh;  ///< hydroMesh containing the original body mesh and the clipping process
    double m_mass;                             ///< mass for which the displacement/equilibrium is solved
    Position m_COG;                            ///< COG for which the equilibrium is solved, in body reference frame

    // Newton-Raphson quantities
    Vector3d<double> m_residual;                ///< residual of the Newton-Raphson solver
    Vector3d<double> m_solution;                ///< solution of the Newton-Raphson solver

    unsigned int m_iteration = 0;               ///< number of Newton-Raphson iterations
    unsigned int m_iterations_max = 100;        ///< number of maximum Newton-Raphson iterations
    double m_relative_tolerance = 1e-2;         ///< relative tolerance for stopping the Newton-Raphson solver

    Vector3d<double> m_relax;                   ///< relaxation vector, to ensure that the evolution of the solution is
    ///< not to large

  };

  /// maker of the hydrostatic equilibrium solver
  /// \param body body for which is solved the hydrostatic equilibrium
  /// \param meshFile name of the file containing the mesh
  /// \param meshOffset mesh frame offset, relatively to the body reference frame
  /// \param mass mass for which the displacement/equilibrium is solved
  /// \param COGPosInBody position of the center of gravity, in body reference frame, for which the equilibrium is solved
  /// \param fc frame convention (NED/NWU)
  /// \return FrHydrostaticEquilibrium object, containing hydrostatic quantities and solver parameters
  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset,
                                double mass,
                                const Position &COGPosInBody,
                                FRAME_CONVENTION fc);


} //end namespace frydom


#endif //FRYDOM_FRHYDROSTATICEQUILIBRIUM_H
