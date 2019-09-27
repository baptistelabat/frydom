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


#ifndef FRYDOM_FRNONLINEARHYDROSTATICFORCE_H
#define FRYDOM_FRNONLINEARHYDROSTATICFORCE_H

#include <frydom/core/force/FrForce.h>
#include "frydom/core/math/FrVector.h"

namespace frydom {

  class FrBody;

  class FrHydroMesh;

  /// This class defines the nonlinear hydrostatic force applied to a hydrodynamic body.
  /// The non linear hydrostatic force is computed by integrating the hydrostatic pressure (-rho g z) over the wetted
  /// body surface. This surface is defined using the body mesh, clipped by the free surface if the body is floating.
  /// 2 levels of approximation are defined for the free surface position
  /// - the mean water level : z = 0
  /// - the incident wave position : z = eta_i
  /// Be careful that in the second approximation, the hydrostatic force is no longer equals to rho g Vdisp vec(z)
  /// and contains some horizontal components. The hydrostatic force also does not apply on the buoyancy center, B,
  /// which means M_B (HS) != 0 (see technical report : non linear hydrostatics and Froude-Krylov)

  /**
   * \class FrNonlinearHydrostaticForce
   * \brief Class for computing nonlinear hydrostatic loads.
   */
  class FrNonlinearHydrostaticForce : public FrForce {

   private:

    /// Hydrodynamic mesh.
    std::shared_ptr<FrHydroMesh> m_hydroMesh;

   public:

    /// Constructor.
    FrNonlinearHydrostaticForce(const std::string& name, const std::shared_ptr<FrHydroMesh> &hydroMesh);

    /// Get the type name of this object
    /// \return type name of this object
    std::string GetTypeName() const override { return "NonlinearHydrostaticForce"; }

    /// Return true if the force is included in the static analysis
    bool IncludedInStaticAnalysis() const override { return true; }

    /// Initialize the log
    void AddFields() override;

    /// Get the center of buoyancy position of the clipped mesh in the body reference frame
    /// Be careful that the center of buoyancy definition is not valid for a mesh clipped by the incident wave
    /// \param fc frame convention (NED/NWU)
    /// \return center of buoyancy position in the body reference frame
    Position GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc);

    /// Get the center of buoyancy position of the clipped mesh in the world reference frame
    /// Be careful that the center of buoyancy definition is not valid for a mesh clipped by the incident wave
    /// \param fc frame convention (NED/NWU)
    /// \return center of buoyancy position in the world reference frame
    Position GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc);

    /// Get the hydrostatic force (integration of the hydrostatic pressure on the clipped mesh)
    /// in the body reference frame
    /// \param fc frame convention (NED/NWU)
    /// \return hydrostatic force in the body reference frame
    Force GetHydrostaticForceInBody(FRAME_CONVENTION fc);

    /// Get the hydrostatic force (integration of the hydrostatic pressure on the clipped mesh)
    /// in the world reference frame
    /// \param fc frame convention (NED/NWU)
    /// \return hydrostatic force in the world reference frame
    Force GetHydrostaticForceInWorld(FRAME_CONVENTION fc);

    /// Get the hydrostatic torque (integration of the hydrostatic pressure on the clipped mesh)
    /// in the world reference frame, at the center of the clipped mesh
    /// \param fc frame convention (NED/NWU)
    /// \return hydrostatic torque in the word reference frame, at the center of the clipped mesh
    Torque GetHydrostaticTorqueInBody(FRAME_CONVENTION fc);

    /// Get the hydrostatic torque (integration of the hydrostatic pressure on the clipped mesh)
    /// in the world reference frame, at the center of the clipped mesh
    /// \param fc frame convention (NED/NWU)
    /// \return hydrostatic torque in the word reference frame, at the center of the clipped mesh
    Torque GetHydrostaticTorqueInWorld(FRAME_CONVENTION fc);

    Position GetMetacentricHeight();

   private:

    /// Update nonlinear hydrostatic force.
    /// \param time Current time of the simulation from beginning.
    void Compute(double time) override;

  };

  /// This function creates a (fully or weakly) nonlinear hydrostatic force object.
  std::shared_ptr<FrNonlinearHydrostaticForce>
  make_nonlinear_hydrostatic_force(const std::string& name,
                                   const std::shared_ptr<FrBody> &body,
                                   const std::shared_ptr<FrHydroMesh> &HydroMesh);

}  // end namespace frydom

#endif //FRYDOM_FRNONLINEARHYDROSTATICFORCE_H
