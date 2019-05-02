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
//#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"

namespace frydom {

    void FrLinearHydrostaticForce::Initialize() {

        // This function initializes the hydrostatic force object.

        // Initialization of the parent class.
        FrForce::Initialize();

        // Equilibrium frame of the body.
        m_equilibriumFrame = m_HDB->GetMapper()->GetEquilibriumFrame(m_body);

        // 3x3 hydrostatic matrix.
        if(HydrostaticsMatrixHDB5) {
            m_stiffnessMatrix.SetData(m_HDB->GetBody(m_body)->GetHydrostaticStiffnessMatrix());
        }

    }

    void FrLinearHydrostaticForce::Compute(double time) {

        // This function computes the linear hydrostatic loads.

        // Body frame.
        auto bodyFrame = m_body->GetFrameAtCOG(NWU);

        // Transformation from the body frame to equilibrium frame.
        auto deltaFrame = m_equilibriumFrame->GetInverse() * bodyFrame;

        // Position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
        mathutils::Vector3d<double> state; double temp;
        state[0] = deltaFrame.GetPosition(NWU).z(); // Vertical position.

        // Angular position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
        deltaFrame.GetRotation().GetCardanAngles_RADIANS(state[1], state[2], temp, NWU);

        // Fh = -Kh*X. in the equilibrium frame: only heave, roll and pitch are considered here.
        auto forceState = - (m_stiffnessMatrix * state); // m_stiffnessMatrix is a 3x3 matrix.

        // Linear hydrostatic force: assumed in the world frame.
        auto worldForce = Force(0., 0., forceState[0]); // Only the heave component is used from forceState, so the first one.
        worldForce.z() += m_body->GetSystem()->GetGravityAcceleration() * m_body->GetMass(); // WARNING: It is assumed that the displacement is equal to the mass, which can be false.
        SetForceInWorldAtCOG( worldForce, NWU);

        // Linear hydrostatic torque: assumed in the body frame/
        auto localTorque = Torque(forceState[1], forceState[2], 0.);
        SetTorqueInBodyAtCOG(localTorque, NWU);
    }

    void FrLinearHydrostaticForce::StepFinalize() {
        FrForce::StepFinalize();
    }

    void FrLinearHydrostaticForce::SetStiffnessMatrix(FrLinearHydrostaticStiffnessMatrix HydrostaticMatrix) {

        // This function sets the hydrostatic stiffness matrix.

        m_stiffnessMatrix = HydrostaticMatrix;
        HydrostaticsMatrixHDB5 = false;
    };

    void FrLinearHydrostaticForce::SetStiffnessMatrix(mathutils::MatrixMN<double> HydrostaticMatrix) {

        // This function sets the hydrostatic stiffness matrix.

        m_stiffnessMatrix.SetK33(HydrostaticMatrix(0, 0));
        m_stiffnessMatrix.SetK44(HydrostaticMatrix(1, 1));
        m_stiffnessMatrix.SetK55(HydrostaticMatrix(2, 2));
        m_stiffnessMatrix.SetK34(HydrostaticMatrix(0, 1));
        m_stiffnessMatrix.SetK35(HydrostaticMatrix(0, 2));
        m_stiffnessMatrix.SetK45(HydrostaticMatrix(1, 2));
        HydrostaticsMatrixHDB5 = false;
    };

    std::shared_ptr<FrLinearHydrostaticForce>
    make_linear_hydrostatic_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body){

        // This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a hydrostatic stiffness matrix given by the hdb.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrLinearHydrostaticForce>(HDB);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }

    std::shared_ptr<FrLinearHydrostaticForce>
    make_linear_hydrostatic_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body, std::string meshfile,Position MeshOffset, mathutils::Matrix33<double> Rotation){

        //FIXME !!!

        // This function creates the linear hydrostatic force object for computing the linear hydrostatic loads with a hydrostatic sitffness matrix computed by FrMesh.

        // Construction of the hydrostatic force object from the HDB.
        auto forceHst = std::make_shared<FrLinearHydrostaticForce>(HDB);

        // Add the hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        // Computation of the hydrostatic stiffness matrix.
        mesh::FrMesh Mesh_Init = mesh::FrMesh(meshfile);
        mesh::FrMeshClipper Mesh_clipper = mesh::FrMeshClipper();
        double TidalHeight = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetTidal()->GetHeight(NWU);
        Mesh_clipper.SetPlaneClippingSurface(TidalHeight);
//        Mesh_clipper.SetBody(body.get());
//        Mesh_clipper.SetMeshOffsetRotation(MeshOffset, Rotation);
//        mesh::FrMesh Clipped_mesh = Mesh_clipper.Apply(Mesh_Init);
//        Position BodyCoG = body->GetCOGPositionInWorld(NWU);
//        Vector3d<double> cog;
//        cog[0] = BodyCoG[0];
//        cog[1] = BodyCoG[1];
//        cog[2] = BodyCoG[2];
//        FrHydrostaticsProperties hsp(HDB->GetWaterDensity(),HDB->GetGravityAcc(),Clipped_mesh,cog);
//        hsp.Process();
//        forceHst->SetStiffnessMatrix(hsp.GetHydrostaticMatrix());
//        Clipped_mesh.Write("Mesh_used_for_Hydrostatic_stiffness_matrix.obj");
//
//        //FIXME: Si la position du corps est mise a jour, un update des forces sera applique (UpdateAfterMove)
//        // qui engendre un bug car la force n'a pas ete initialisee.
//        // D'ou l'initialize ci-dessous.
//        forceHst->Initialize();

        return forceHst;
    }

}  // end namespace frydom
