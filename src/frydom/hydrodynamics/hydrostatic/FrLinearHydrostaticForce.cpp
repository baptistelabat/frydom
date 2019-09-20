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

namespace frydom {

    template<typename OffshoreSystemType>
    FrLinearHydrostaticForce<OffshoreSystemType>::FrLinearHydrostaticForce(const std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> &eqFrame) :
        m_equilibriumFrame(eqFrame.get()) {}

    template<typename OffshoreSystemType>
    FrLinearHydrostaticForce<OffshoreSystemType>::FrLinearHydrostaticForce(FrEquilibriumFrame<OffshoreSystemType> *eqFrame) :
        m_equilibriumFrame(eqFrame) {}

    template<typename OffshoreSystemType>
    void FrLinearHydrostaticForce<OffshoreSystemType>::Initialize() {

      // This function initializes the hydrostatic force object.

      // Initialization of the parent class.
      FrForce<OffshoreSystemType>::Initialize();

    }

    template<typename OffshoreSystemType>
    void FrLinearHydrostaticForce<OffshoreSystemType>::Compute(double time) {

      // This function computes the linear hydrostatic loads.

      // Body frame.
      auto bodyFrame = this->m_body->GetFrameAtCOG(NWU);

      // Transformation from the body frame to equilibrium frame.
      auto deltaFrame = m_equilibriumFrame->GetInverse() * bodyFrame;

      // Position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
      mathutils::Vector3d<double> state;
      double temp;
      state[0] = deltaFrame.GetPosition(NWU).z(); // Vertical position.

      // Angular position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
      deltaFrame.GetRotation().GetCardanAngles_RADIANS(state[1], state[2], temp, NWU);

      // Fh = -Kh*X. in the equilibrium frame: only heave, roll and pitch are considered here.
      auto forceState = -(m_stiffnessMatrix * state); // m_stiffnessMatrix is a 3x3 matrix.

      // Linear hydrostatic force: assumed in the world frame.
      auto worldForce = Force(0., 0.,
                              forceState[0]); // Only the heave component is used from forceState, so the first one.
      worldForce.z() += this->m_body->GetSystem()->GetGravityAcceleration() *
                        this->m_body->GetMass(); // WARNING: It is assumed that the displacement is equal to the mass, which can be false.
      this->SetForceInWorldAtCOG(worldForce, NWU);

      // Linear hydrostatic torque: assumed in the body frame/
      auto localTorque = Torque(forceState[1], forceState[2], 0.);
      this->SetTorqueInBodyAtCOG(localTorque, NWU);
    }

    template<typename OffshoreSystemType>
    FrLinearHydrostaticStiffnessMatrix FrLinearHydrostaticForce<OffshoreSystemType>::GetStiffnessMatrix() const {
      return m_stiffnessMatrix;
    }

    template<typename OffshoreSystemType>
    void FrLinearHydrostaticForce<OffshoreSystemType>::SetStiffnessMatrix(FrLinearHydrostaticStiffnessMatrix HydrostaticMatrix) {

      // This function sets the hydrostatic stiffness matrix.

      m_stiffnessMatrix = HydrostaticMatrix;
      HydrostaticsMatrixHDB5 = false;
    };

    template<typename OffshoreSystemType>
    void FrLinearHydrostaticForce<OffshoreSystemType>::SetStiffnessMatrix(mathutils::MatrixMN<double> HydrostaticMatrix) {

      // This function sets the hydrostatic stiffness matrix.

      m_stiffnessMatrix.SetK33(HydrostaticMatrix(0, 0));
      m_stiffnessMatrix.SetK44(HydrostaticMatrix(1, 1));
      m_stiffnessMatrix.SetK55(HydrostaticMatrix(2, 2));
      m_stiffnessMatrix.SetK34(HydrostaticMatrix(0, 1));
      m_stiffnessMatrix.SetK35(HydrostaticMatrix(0, 2));
      m_stiffnessMatrix.SetK45(HydrostaticMatrix(1, 2));
      HydrostaticsMatrixHDB5 = false;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearHydrostaticForce<OffshoreSystemType>>
    make_linear_hydrostatic_force(const std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> &eqFrame,
                                  const std::shared_ptr<FrBody<OffshoreSystemType>> &body) {

      // Construction of the hydrostatic force object from the HDB.
      auto forceHst = std::make_shared<FrLinearHydrostaticForce>(eqFrame);

      // Add the hydrostatic force object as an external force to the body.
      body->AddExternalForce(forceHst);

      return forceHst;

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearHydrostaticForce<OffshoreSystemType>>
    make_linear_hydrostatic_force(const std::shared_ptr<FrHydroDB<OffshoreSystemType>> &HDB, const std::shared_ptr<FrBody<OffshoreSystemType>> &body) {

      //TODO : use first constructor, once GetMapper->GetEquilibriumFrame will return a shared pointer

      // This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a hydrostatic stiffness matrix given by the hdb.

      // Construction of the hydrostatic force object from the HDB.
      auto forceHst = std::make_shared<FrLinearHydrostaticForce>(HDB->GetMapper()->GetEquilibriumFrame(body.get()));

      // Add the hydrostatic force object as an external force to the body.
      body->AddExternalForce(forceHst);

      forceHst->SetStiffnessMatrix(HDB->GetBody(body)->GetHydrostaticStiffnessMatrix());

      return forceHst;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearHydrostaticForce<OffshoreSystemType>>
    make_linear_hydrostatic_force(const std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> &eqFrame,
                                  const std::shared_ptr<FrBody<OffshoreSystemType>> &body,
                                  const std::string &meshFile, FrFrame meshOffset) {

      // This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a
      // hydrostatic stiffness matrix computed by FrMesh.
      auto forceHst = make_linear_hydrostatic_force(eqFrame, body);

      // Create a hydroMesh, to set up the mesh in the body frame and then clip it
      auto hydroMesh = make_hydro_mesh(body, meshFile, meshOffset, FrHydroMesh<OffshoreSystemType>::ClippingSupport::PLANESURFACE);
      hydroMesh->Initialize();
      hydroMesh->Update(0.);

      // To check the clipped mesh, uncomment the following line
      hydroMesh->GetClippedMesh().Write("Clipped_Mesh.obj");

      // Compute all hydrostatics properties and files a report
      FrHydrostaticsProperties hsp(body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER),
                                   body->GetSystem()->GetGravityAcceleration(),
                                   hydroMesh->GetClippedMesh(),
                                   body->GetCOGPositionInWorld(NWU));
      hsp.Process();

      // For visualizing the report, uncomment the following line
      std::cout << hsp.GetReport() << std::endl;

      // Set the stiffness matrix
      forceHst->SetStiffnessMatrix(hsp.GetHydrostaticMatrix());

      // Remove the temporary hydroMesh from the system
      body->GetSystem()->RemovePhysicsItem(hydroMesh);

      //FIXME: Si la position du corps est mise a jour, un update des forces sera applique (UpdateAfterMove)
      // qui engendre un bug car la force n'a pas ete initialisee.
      // D'ou l'initialize ci-dessous.
      forceHst->Initialize();

      return forceHst;
    }

}  // end namespace frydom
