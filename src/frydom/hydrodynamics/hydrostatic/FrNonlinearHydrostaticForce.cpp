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

        // Loading the input mesh file.
        m_mesh_init = mesh::FrMesh(meshfilename);

    }

    void FrNonlinearHydrostaticForce::InitializeLog(){

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("Center of buoyancy","m", fmt::format("Center of buoyancy in world reference frame in {}", c_logFrameConvention),
                 [this]() {return GetCoB();});

        FrForce::InitializeLog();

    }

    void FrNonlinearHydrostaticForce::Update(double time) {

        // This function computes the nonlinear hydrostatic loads.

        // Loading the input mesh file.
        mesh::FrMesh current_mesh = m_mesh_init;

        // Body linear and angular position.
        Position PositionOfBodyInWorld = m_body->GetPosition(NWU);
        Position PositionOfCoGInWorld = m_body->GetCOGPositionInWorld(NWU);
        double phi, theta, psi;
        m_body->GetRotation().GetCardanAngles_RADIANS(phi,theta,psi, NWU);

        // Transport of the mesh at its good position and updates its orientation.
        VectorT<double, 3> trans;

        // Translation. of -OG.
        trans[0] = -PositionOfCoGInWorld.GetX();
        trans[1] = -PositionOfCoGInWorld.GetY();
        trans[2] = -PositionOfCoGInWorld.GetZ();
        current_mesh.Translate(trans);

        // Rotation around the CoG of the body.
        current_mesh.Rotate(phi,theta,psi);

        // Translation. of +OG.
        trans[0] = PositionOfCoGInWorld.GetX();
        trans[1] = PositionOfCoGInWorld.GetY();
        trans[2] = PositionOfCoGInWorld.GetZ();
        current_mesh.Translate(trans);

        // Translation.
        trans[0] = PositionOfBodyInWorld.GetX();
        trans[1] = PositionOfBodyInWorld.GetY();
        trans[2] = PositionOfBodyInWorld.GetZ();
        current_mesh.Translate(trans);

        // Clipper.
        mesh::MeshClipper clipper;

        // Tidal height.
        double TidalHeight = m_system->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);

        // Ocean.
        FrFreeSurface* FreeSurface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();

        // Clipping surface.
        mesh::ClippingWaveSurface clippingSurface = clipper.SetWaveClippingSurface(TidalHeight,FreeSurface);

        // Clipping.
        m_clipped_mesh = clipper(current_mesh);

        // Computation of the hydrostatic force.
        NonlinearHydrostatics NLhydrostatics(m_HDB->GetWaterDensity(),m_HDB->GetGravityAcc()); // Creation of the NonlinearHydrostatics structure.
        NLhydrostatics.CalcPressureIntegration(m_clipped_mesh);

        // Setting the nonlinear loads in world at the CoB.
        Force force = NLhydrostatics.GetWeaklyNonlinearForce();
        m_CoB = NLhydrostatics.GetCenterOfBuoyancy();
        this->SetForceInWorldAtPointInWorld(force,m_CoB,NWU); // The torque is computed from the hydrostatic force and the center of buoyancy.

    }

    void FrNonlinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();

        // Writing the clipped mesh in an output file.
//        m_clipped_mesh.Write("Mesh_clipped.obj");

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