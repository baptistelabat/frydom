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


#include "FrLinearHydrostaticForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

#include "frydom/mesh/FrHydroMesh.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  FrLinearHydrostaticForce::FrLinearHydrostaticForce(const std::string &name,
                                                     FrBody *body,
                                                     const std::shared_ptr<FrEquilibriumFrame> &eqFrame) :
      FrForce(name, TypeToString(this), body),
      m_equilibriumFrame(eqFrame) {}

  void FrLinearHydrostaticForce::Initialize() {

    // This function initializes the hydrostatic force object.

    // Initialization of the parent class.
    FrForce::Initialize();

  }

  void FrLinearHydrostaticForce::Compute(double time) {

    auto body = GetBody();

    // This function computes the linear hydrostatic loads.

    // Delta frame
    auto deltaFrame = m_equilibriumFrame->GetPerturbationFrame();

    // Position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
    mathutils::Vector3d<double> state;
    double temp;
    state[0] = deltaFrame.GetPosition(NWU).z(); // Vertical position.

    // Angular position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
    deltaFrame.GetRotation().GetCardanAngles_RADIANS(state[1], state[2], temp, NWU);

    // Fh = -Kh*X. in the equilibrium frame: only heave, roll and pitch are considered here.
    mathutils::Vector3d<double> forceState = -(m_stiffnessMatrix * state); // m_stiffnessMatrix is a 3x3 matrix.

    // Linear hydrostatic force: assumed in the world frame.
    auto worldForce = Force(0., 0.,
                            forceState[0]); // Only the heave component is used from forceState, so the first one.
    // WARNING: It is assumed that the displacement is equal to the mass, which can be false.
    worldForce.z() += body->GetSystem()->GetGravityAcceleration() * body->GetMass();
    SetForceInWorldAtCOG(worldForce, NWU);

    // Linear hydrostatic torque: assumed in the body frame/
    auto localTorque = Torque(forceState[1], forceState[2], 0.);
    SetTorqueInBodyAtCOG(localTorque, NWU);
  }

  FrLinearHydrostaticStiffnessMatrix FrLinearHydrostaticForce::GetStiffnessMatrix() const {
    return m_stiffnessMatrix;
  }

  void FrLinearHydrostaticForce::SetStiffnessMatrix(FrLinearHydrostaticStiffnessMatrix HydrostaticMatrix) {

    // This function sets the hydrostatic stiffness matrix.

    m_stiffnessMatrix = HydrostaticMatrix;
  };

  void FrLinearHydrostaticForce::SetStiffnessMatrix(mathutils::MatrixMN<double> HydrostaticMatrix) {

    // This function sets the hydrostatic stiffness matrix.

    m_stiffnessMatrix.SetK33(HydrostaticMatrix(0, 0));
    m_stiffnessMatrix.SetK44(HydrostaticMatrix(1, 1));
    m_stiffnessMatrix.SetK55(HydrostaticMatrix(2, 2));
    m_stiffnessMatrix.SetK34(HydrostaticMatrix(0, 1));
    m_stiffnessMatrix.SetK35(HydrostaticMatrix(0, 2));
    m_stiffnessMatrix.SetK45(HydrostaticMatrix(1, 2));
  }

  std::shared_ptr<FrLinearHydrostaticForce>
  make_linear_hydrostatic_force(const std::string &name,
                                const std::shared_ptr<FrBody> body,
                                const std::shared_ptr<FrEquilibriumFrame> eqFrame) {

    // Construction of the hydrostatic force object from the HDB.
    auto forceHst = std::make_shared<FrLinearHydrostaticForce>(name, body.get(), eqFrame);

    // Add the hydrostatic force object as an external force to the body.
    body->AddExternalForce(forceHst);

    return forceHst;

  }

  std::shared_ptr<FrLinearHydrostaticForce>
  make_linear_hydrostatic_force(const std::string &name,
                                std::shared_ptr<FrBody> body,
                                std::shared_ptr<FrHydroDB> HDB) {

    // This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a
    // hydrostatic stiffness matrix given by the hdb.

    // Construction of the hydrostatic force object from the HDB.
    auto forceHst = make_linear_hydrostatic_force(name,
                                                  body,
                                                  HDB->GetMapper()->GetSharedEquilibriumFrame(body.get()));

    forceHst->SetStiffnessMatrix(HDB->GetBody(body)->GetHydrostaticStiffnessMatrix());

    return forceHst;
  }

  std::shared_ptr<FrLinearHydrostaticForce>
  make_linear_hydrostatic_force(const std::string &name,
                                std::shared_ptr<FrBody> body,
                                std::shared_ptr<FrEquilibriumFrame> eqFrame,
                                const std::string &meshFile,
                                FrFrame meshOffset) {

    // This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a
    // hydrostatic stiffness matrix computed by FrMesh.
    auto forceHst = make_linear_hydrostatic_force(name, body, eqFrame);

    // Create a hydroMesh, to set up the mesh in the body frame and then clip it
    auto hydroMesh = make_hydro_mesh("mesh" + name,
                                     body,
                                     meshFile,
                                     meshOffset,
                                     FrHydroMesh::ClippingSupport::PLANESURFACE);

    hydroMesh->Initialize();
    hydroMesh->Update(0.);

    // To check the clipped mesh, uncomment the following line
    hydroMesh->GetClippedMesh().Write("Clipped_Mesh.obj");

    // Compute all hydrostatics properties and files a report
    FrHydrostaticsProperties hsp(body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                 body->GetSystem()->GetGravityAcceleration(),
                                 hydroMesh->GetClippedMesh(),
                                 body->GetCOGPositionInWorld(NWU), NWU);
    hsp.ComputeProperties();

    // For visualizing the report, uncomment the following line
//      std::cout << hsp.GetReport() << std::endl;

    // Set the stiffness matrix
    forceHst->SetStiffnessMatrix(hsp.GetHydrostaticMatrix());

    // Remove the temporary hydroMesh from the system
    body->GetSystem()->Remove(hydroMesh);

    //FIXME: Si la position du corps est mise a jour, un update des forces sera applique (UpdateAfterMove)
    // qui engendre un bug car la force n'a pas ete initialisee.
    // D'ou l'initialize ci-dessous.
    forceHst->Initialize();

    return forceHst;
  }

}  // end namespace frydom
