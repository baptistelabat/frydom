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

#include "FrNonlinearHydrostaticForce.h"

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    void FrNonlinearHydrostaticForce::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce::Initialize();

    }

    void FrNonlinearHydrostaticForce::Update(double time) {

        // This function computes the nonlinear hydrostatic loads.

        // Loading the input mesh file.
        mesh::FrMesh mesh(meshfilename);

        // Body linear and angular position.
        Position PositionOfBodyInWorld = m_body->GetPosition(NWU);
        FrRotation CardanAnglesBody = m_body->GetRotation();
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

        // Clipper.
        mesh::MeshClipper clipper;

        // Tidal height.
        double TidalHeight = m_system->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);

        // Ocean.
        FrFreeSurface* FreeSurface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();

        // Clipping surface.
        mesh::ClippingWaveSurface clippingSurface = clipper.SetWaveClippingSurface(TidalHeight,FreeSurface);

        // Clipping.
        mesh::FrMesh mesh_clipped = clipper(mesh);

        // Computation of the hydrostatic force.
        NonlinearHydrostatics NLhydrostatics(m_HDB->GetWaterDensity(),m_HDB->GetGravityAcc()); // Creation of the NonlinearHydrostatics structure.
        NLhydrostatics.CalcPressureIntegration(mesh_clipped);

        // Writing the clipped mesh in an output file.
        mesh_clipped.Write("Mesh_clipped.obj");

        // Setting the weakly nonlinear loads in world.
        Force force = NLhydrostatics.GetWeaklyNonlinearForce();
        this->SetForceInWorldAtPointInWorld(force,NLhydrostatics.GetCenterOfBuoyancy(),NWU); // The torque is computed from the hydrostatic force and the center of buoyancy.

    }

    void FrNonlinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();
    }

    std::shared_ptr<FrNonlinearHydrostaticForce>
    make_nonlinear_hydrostatic_force(FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile){

        // This function creates the nonlinear hydrostatic force object for computing the nonlinear hydrostatic loads.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrNonlinearHydrostaticForce>(system,HDB,meshfile);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

}  // end namespace frydom