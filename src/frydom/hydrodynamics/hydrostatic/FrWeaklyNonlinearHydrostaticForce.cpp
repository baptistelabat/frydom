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

#include "FrWeaklyNonlinearHydrostaticForce.h"

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"

namespace frydom {

    void FrWeaklyNonlinearHydrostaticForce_::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce_::Initialize();

        // Equilibrium frame of the body.
        m_equilibriumFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

    }

    void FrWeaklyNonlinearHydrostaticForce_::Update(double time) {

        // This function computes the weakly nonlinear hydrostatic loads.

        // Loading the input mesh file.
        mesh::FrMesh mesh(meshfilename);

        // Body linear and angular position.
        Position PositionOfBodyInWorld = m_body->GetPosition(NWU);
        FrRotation_ CardanAnglesBody = m_body->GetRotation();
        double phi, theta, psi;
        CardanAnglesBody.GetCardanAngles_RADIANS(phi, theta, psi, NWU);

        // Transport of the mesh at its good position and updates its orientation.

        // Translation.
        VectorT<double, 3> trans;
        trans[0] = PositionOfBodyInWorld.GetX();
        trans[1] = PositionOfBodyInWorld.GetY();
        trans[2] = PositionOfBodyInWorld.GetZ();
        mesh.Translate(trans);

        // Rotation.
        mesh.Rotate(phi,theta,psi);

        // Clipping and computation of the hydrostatic force.
        NonlinearHydrostatics NLhydrostatics(m_HDB->GetWaterDensity(),m_HDB->GetGravityAcc()); // Creation of the NonlinearHydrostatics structure.
        NLhydrostatics.Load(mesh); // The mesh clipping and the hydrostatic computation are performed here.

        // Writting the clipped mesh in an output file.
        mesh::FrMesh mesh_clipped = NLhydrostatics.GetClippedMesh();
        mesh_clipped.Write("Mesh_clipped.obj");

        // Setting the weakly nonlinear loads in world.
        Force force = NLhydrostatics.GetWeaklyNonlinearForce();
        this->SetForceInWorldAtPointInWorld(force,NLhydrostatics.GetCenterOfBuoyancy(),NWU); // The torque is computed from the hydrostatic force and the center of buoyancy.

    }

    void FrWeaklyNonlinearHydrostaticForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }

    std::shared_ptr<FrWeaklyNonlinearHydrostaticForce_>
    make_weakly_nonlinear_hydrostatic_force(std::shared_ptr<FrHydroDB_> HDB, std::shared_ptr<FrBody_> body, std::string meshfile){

        // This function creates the weakly nonlinear hydrostatic force object for computing the weakly nonlinear hydrostatic loads.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrWeaklyNonlinearHydrostaticForce_>(HDB,meshfile);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom