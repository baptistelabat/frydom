//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "FrNode.h"
#include "FrHydroBody.h"
#include "frydom/core/FrNodeDynamic.h"

namespace frydom {

    FrHydroBody::FrHydroBody() : is3DOF(false),
                                FrBody()
    {
        //variablesHydro.Initialize(variables);
        //variables.SetDisabled(true);
        //variablesHydro.SetDisabled(false);
        variables_ptr = &variables;
        m_properties = std::make_unique<FrHydroBodyProperties>();
    }

    void FrHydroBody::Set3DOF(const bool flag) {
        if (flag) {
            Set3DOF_ON();
        } else {
            Set3DOF_OFF();
        }
    }

    void FrHydroBody::Set3DOF_ON() {
        if (is3DOF){ return; }
        // TODO: voir si on peut pas faire quelque chose avec l'attribut flipped de ChLinkMate...

        try {
            if (!GetSystem()) {
                throw std::string("The body must be added to a system before the plane constraint is set");
            }
//            if (!GetSystem()->getFreeSurface()) {
//                throw std::string("A free surface has to be created before setting a plane constraint for a body");
//            }
        } catch(std::string const& msg) {
            std::cerr << msg << std::endl;
        }

        auto plane_constraint = std::make_shared<chrono::ChLinkMatePlane>();
        auto free_surface_body = GetSystem()->GetEnvironment()->GetFreeSurface()->GetBody();
        plane_constraint->Initialize(shared_from_this(), free_surface_body,
                                     true,
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(0, 0, 1),
                                     chrono::ChVector<>(0, 0, -1));
        system->AddLink(plane_constraint);
        constraint3DOF = plane_constraint;
        is3DOF = true;
    }

    void FrHydroBody::Set3DOF_OFF() {
        if (!is3DOF) { return; }

        system->RemoveLink(constraint3DOF);
        constraint3DOF->SetSystem(0);
        is3DOF = false;
    }

    void FrHydroBody::Set3DOF_ON(chrono::ChVector<> dir) {
        if (is3DOF) { return; }

        try {
            if (!GetSystem()) {
                throw std::string("The body must be added to a system before the plane constraint is set");
            }
        } catch(std::string const& msg) {
            std::cerr << msg << std::endl;
        }

        auto plane_constraint = std::make_shared<chrono::ChLinkMatePlane>();
        auto free_surface_body = GetSystem()->GetEnvironment()->GetFreeSurface()->GetBody();
        plane_constraint->Initialize(shared_from_this(), free_surface_body,
                                     true,
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(),
                                     dir, -dir );
        system->AddLink(plane_constraint);
        constraint3DOF = plane_constraint;
        is3DOF = true;
    }

    void FrHydroBody::Set3DOF_ON(chrono::ChVector<> dir,
                                 chrono::ChVector<> pos1,
                                 chrono::ChVector<> pos2) {
        if (is3DOF) { return; }

        try {
            if (!GetSystem()) {
                throw std::string("The body must be added to a system before the plane constraint is set");
            }
        } catch(std::string const& msg) {
            std::cerr << msg << std::endl;
        }

        auto plane_constraint = std::make_shared<chrono::ChLinkMatePlane>();
        auto free_surface_body = GetSystem()->GetEnvironment()->GetFreeSurface()->GetBody();
        plane_constraint->Initialize(shared_from_this(), free_surface_body,
                                     true, pos1, pos2, dir, -dir );
        system->AddLink(plane_constraint);
        constraint3DOF = plane_constraint;
        is3DOF = true;
    }


    void FrHydroBody::SetDOF(bool mc_x, bool mc_y, bool mc_z,
                             bool mc_rx, bool mc_ry, bool mc_rz) {

        try {
            if (!GetSystem()) {
                throw std::string("The body must be added to a system before the plane constraint is set");
            }
        } catch(std::string const& msg) {
            std::cerr << msg << std::endl;
        }

        auto free_surface_body = GetSystem()->GetEnvironment()->GetFreeSurface()->GetBody();
        auto constraint = std::make_shared<chrono::ChLinkMateGeneric>(mc_x, mc_y, mc_z, mc_rx, mc_ry, mc_rz);
        constraint->Initialize(shared_from_this(), free_surface_body,
                               true,
                               //chrono::ChFrame<>(chrono::ChVector<>()),
                               GetFrame_REF_to_COG(),
                               chrono::ChFrame<>(chrono::ChVector<>()));
        system->AddLink(constraint);
        }


    void FrHydroBody::SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset) {

        m_hydro_mesh = mesh;

        if (as_asset) {
            // Registering the mesh as an asset
            auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
            shape->SetMesh(*mesh);
            AddAsset(shape);
        }

        // TODO: Ajouter automatiquement un clipper
    }

    void FrHydroBody::SetHydroMesh(std::string obj_filename, bool as_asset) {
        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        SetHydroMesh(mesh, as_asset);
    }

    void FrHydroBody::Update(bool update_assets) {

//        std::cout << "Updating body"  << std::endl;

        // Update heading
        auto euler_angles = quat_to_euler(GetRot(), CARDAN, RAD);
        m_heading = Normalize_0_2PI(euler_angles.z());

        // Update course
        auto body_velocity = GetVelocity(NWU);
        m_course = Normalize_0_2PI(atan2(body_velocity.y(), body_velocity.x()));

        // Update current relative velocity
        auto current_velocity = GetSystem()->GetEnvironment()->GetCurrent()->GetFluxVector(NWU);

        m_current_relative_velocity = body_velocity - current_velocity;

        // Update the current relative angle
        auto relative_velocity_angle = atan2(m_current_relative_velocity.y(),
                                             m_current_relative_velocity.x());
        m_current_relative_angle = Normalize__PI_PI(relative_velocity_angle - m_heading);

        // update parent class
        chrono::ChBodyAuxRef::Update(update_assets);

    }

    // ==========================================================================
    // METHODS ABOUT CURRENT
    // ==========================================================================

    chrono::ChVector<>
    FrHydroBody::GetCurrentRelativeVelocity(FrFrame frame, FrRefSyst Refsys) const {
        auto current_relative_velocity = m_current_relative_velocity;
        if (Refsys == LOCAL) {
            /// Transformation of the DIRECTION
            current_relative_velocity = TransformDirectionParentToLocal(current_relative_velocity);
        }
        switch (frame) {
            case NWU:
                return current_relative_velocity;
            case NED:
                return NWU2NED(current_relative_velocity);
        }
    }

    chrono::ChVector<>
    FrHydroBody::GetCurrentRelativeVelocity(const chrono::ChVector<> &localpos, FrFrame frame, FrRefSyst Refsys) const {
        auto current_velocity = GetSystem()->GetEnvironment()->GetCurrent()->GetFluxVector(NWU);
        auto velocity = PointSpeedLocalToParent(localpos);
        auto current_relative_velocity = current_velocity - velocity;
        if (Refsys == LOCAL) {
            /// Transformation of the DIRECTION
            current_relative_velocity = TransformDirectionParentToLocal(current_relative_velocity);
        }
        switch (frame) {
            case NWU:
                return current_relative_velocity;
            case NED:
                return NWU2NED(current_relative_velocity);
        }
    }

    chrono::ChVector<>
    FrHydroBody::GetCurrentRelativeVelocity(const FrNode* mNode, FrFrame frame, FrRefSyst Refsys) const {
        auto nodeVelocity = PointSpeedLocalToParent(mNode->GetPos());
        auto current_relative_velocity = GetSystem()->GetEnvironment()->GetCurrent()->GetFluxVector(NWU) - nodeVelocity;
        if (Refsys == LOCAL) {
            /// Transformation of the DIRECTION
            current_relative_velocity = TransformDirectionParentToLocal(current_relative_velocity);
            current_relative_velocity = mNode->TransformDirectionParentToLocal(current_relative_velocity);
        }
        switch (frame) {
            case NWU:
                return current_relative_velocity;
            case NED:
                return NWU2NED(current_relative_velocity);
        }
    }

    double FrHydroBody::GetCurrentRelativeAngle(FrFrame frame, ANGLE_UNIT angleUnit) const {
        double angle = m_current_relative_angle;
        if (angleUnit == DEG) {
            angle = angle * RAD2DEG;
        }

        switch (frame) {
            case NWU:
                return angle;
            case NED:
                return -angle;  // TODO: A verifier...
        }

    }

    // ==========================================================================
    // METHODS ABOUT WIND
    // ==========================================================================

    chrono::ChVector<> FrHydroBody::GetWindRelativeVelocity(FrFrame frame, FrRefSyst Refsys) const {
        auto body_velocity = GetVelocity(NWU);
        auto wind_relative_velocity = GetSystem()->GetEnvironment()->GetWind()->GetFluxVector(NWU) - body_velocity;
        if (Refsys == LOCAL) {
            /// Transformation of the DIRECTION
            wind_relative_velocity = TransformDirectionParentToLocal(wind_relative_velocity);
        }
        switch (frame) {
            case NWU:
                return wind_relative_velocity;
            case NED:
                return NWU2NED(wind_relative_velocity);
        }
    }

    chrono::ChVector<> FrHydroBody::GetWindRelativeVelocity(const FrNode* mNode, FrFrame frame, FrRefSyst Refsys) const {
        /// Node velocity computation in the global frame
        auto nodeVelocity = PointSpeedLocalToParent(mNode->GetPos());
        /// Wind relative velocity computed in the global frame
        auto wind_relative_velocity = GetSystem()->GetEnvironment()->GetWind()->GetFluxVector(NWU) - nodeVelocity;
        if (Refsys == LOCAL){
                /// Transformation of the DIRECTION, via frame composition
                wind_relative_velocity = TransformDirectionParentToLocal(wind_relative_velocity);
                wind_relative_velocity = mNode->TransformDirectionParentToLocal(wind_relative_velocity);
        }
        switch (frame) {
            case NWU:
                return wind_relative_velocity;
            case NED:
                return NWU2NED(wind_relative_velocity);
        }
    }

    void FrHydroBody::SetNEDHeading(const double heading_angle, ANGLE_UNIT angleUnit) {
        auto quaternion = euler_to_quat(0., 0., -heading_angle, CARDAN, angleUnit);
        SetRot(quaternion);
    }

    void FrHydroBody::SetNEDHeading(const chrono::ChVector<>& unit_vector) {
        // TODO: verifier qu'on a un vecteur unite
        auto heading_angle = atan2(unit_vector.y(), unit_vector.x());
        FrHydroBody::SetNEDHeading(heading_angle, RAD);
    }

    double FrHydroBody::GetTransverseUnderWaterArea() const {
        return m_properties->GetTransverseUnderWaterArea();
    }

    void FrHydroBody::SetTransverseUnderWaterArea(double area) {
        m_properties->SetTransverseUnderWaterArea(area);
    }

    double FrHydroBody::GetLateralUnderWaterArea() const {
        return m_properties->GetLateralUnderWaterArea();
    }

    void FrHydroBody::SetLateralUnderWaterArea(double area) {
        m_properties->SetLateralUnderWaterArea(area);
    }

    double FrHydroBody::GetTransverseAboveWaterArea() const {
        return m_properties->GetTransverseAboveWaterArea();
    }

    void FrHydroBody::SetTransverseAboveWaterArea(double area) {
        m_properties->SetTransverseAboveWaterArea(area);
    }

    double FrHydroBody::GetLateralAboveWaterArea() const {
        return m_properties->GetLateralAboveWaterArea();
    }

    void FrHydroBody::SetLateralAboveWaterArea(double area) {
        m_properties->SetLateralAboveWaterArea(area);
    }

    double FrHydroBody::GetLpp() const {
        return m_properties->GetLpp();
    }

    void FrHydroBody::SetLpp(double lpp) {
        m_properties->SetLpp(lpp);
    }

    double FrHydroBody::GetWettedSurface() const {
        return m_properties->GetWettedSurface();
    }

    void FrHydroBody::SetWettedSurface(double wetted_surface) {
        m_properties->SetWetterSurface(wetted_surface);
    }

//    void FrHydroBody::SetBEMBody(std::shared_ptr<FrBEMBody> BEMBody) {
//        m_BEMBody = BEMBody;
//        BEMBody->SetHydroBody(this);
//
//        // TODO: Il faut renseigner les masses ajoutees a l'infini !!!!
//
//    }

    void FrHydroBody::SetVariables(const FrVariablesBody vtype) {

        switch (vtype) {
            case variablesStandard:
                break;
            case variablesHydro:
                variables_ptr = new FrVariablesBEMBodyMass();
                GetVariables<FrVariablesBEMBodyMass>()->Initialize(variables);
                break;
        }
    }

    //void FrHydroBody::SetCurrentRefFrameAsEquilibrium() {
    //    auto freeSurfaceFrame = dynamic_cast<FrOffshoreSystem*>(system)->GetEnvironment()->GetFreeSurface()->GetFrame();
    //    chrono::ChFrameMoving<double> eqFrame0 = GetFrame_REF_to_abs() >> freeSurfaceFrame->GetInverse();
    //    SetEquilibriumFrame( dynamic_caststd::shared_ptr<chrono::ChFrameMoving<double>>>(eqFrame0));
    //}

    //chrono::ChFrameMoving<double> FrHydroBody::GetEquilibriumFrame() const {
    //    auto freeSurfaceFrame = dynamic_cast<FrOffshoreSystem*>(system)->GetEnvironment()->GetFreeSurface()->GetFrame();
    //    auto eqFrame = m_equilibriumFrame >> freeSurfaceFrame->GetInverse();
    //    return eqFrame;
    //}

    void FrHydroBody::SetSteadyVelocity(chrono::ChVector<> velocity) {
        m_equilibriumFrame->SetPos_dt(velocity);
    }

    chrono::ChVector<double> FrHydroBody::GetSteadyVelocity() const {
        return m_equilibriumFrame->GetPos_dt();
    }

    void FrHydroBody::SetEquilibriumFrame(const FrEquilibriumFrameType frame,
                                          const double T0, const double psi) {
        assert(frame == DampingSpring);
        auto body_frame = std::make_shared<FrNodeDynamic>(this, T0, psi);
        body_frame->SetPos(GetFrame_REF_to_abs().GetPos());
        system->Add(body_frame);
        m_equilibriumFrame = body_frame;
    }

    void FrHydroBody::SetEquilibriumFrame(const FrEquilibriumFrameType frame,
                                          const double val) {
        assert(frame == MeanMotion);
        auto body_frame = std::make_shared<FrNodeMeanMotion>(this, val);
        body_frame->SetPos(GetFrame_REF_to_abs().GetPos());
        auto z = body_frame->GetPos().z(); // ##CC
        system->Add(body_frame);
        m_equilibriumFrame = body_frame;
    }

    void FrHydroBody::SetEquilibriumFrame(const FrEquilibriumFrameType frame,
                                          const chrono::ChVector<> vect) {
        auto node = CreateNode();
        node->SetPos(vect);
        m_equilibriumFrame = node;
    }



    void FrHydroBody::Initialize() {
        FrBody::Initialize();

        // Initializing message
        // TODO
    }

    void FrHydroBody::VariablesFbIncrementMq() {
        variables_ptr->Compute_inc_invMb_v(variables_ptr->Get_fb(), variables_ptr->Get_qb());
    }

    void FrHydroBody::InjectVariables(chrono::ChSystemDescriptor& mdescriptor) {
        this->variables_ptr->SetDisabled(!this->IsActive());
        mdescriptor.InsertVariables(variables_ptr);
    }

    void FrHydroBody::SetInfiniteAddedMass(const Eigen::MatrixXd& CMInf) {

        auto variablesBEM = dynamic_cast<FrVariablesBEMBodyMass*>(variables_ptr);
        variablesBEM->Initialize(variables);
        variablesBEM->SetInfiniteAddedMass(CMInf);

        variables.SetDisabled(true);
        variables_ptr->SetDisabled(false);
    }

    void FrHydroBody::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                 const chrono::ChStateDelta& v,
                                 const chrono::ChVectorDynamic<>& R,
                                 const unsigned int off_L,  // offset in L, Qc
                                 const chrono::ChVectorDynamic<>& L,
                                 const chrono::ChVectorDynamic<>& Qc) {
        this->variables_ptr->Get_qb().PasteClippedMatrix(v, off_v, 0, 6, 1, 0, 0);  // for solver warm starting only
        this->variables_ptr->Get_fb().PasteClippedMatrix(R, off_v, 0, 6, 1, 0, 0);  // solver known term
    }

    void FrHydroBody::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                   chrono::ChStateDelta& v,
                                   const unsigned int off_L,  // offset in L
                                   chrono::ChVectorDynamic<>& L) {
        v.PasteMatrix(this->variables_ptr->Get_qb(), off_v, 0);
    }

    void FrHydroBody::VariablesFbReset() {
        this->variables_ptr->Get_fb().FillElem(0.0);
    }

    void FrHydroBody::VariablesFbLoadForces(double factor) {
        // add applied forces to 'fb' vector
        this->variables_ptr->Get_fb().PasteSumVector(Xforce * factor, 0, 0);

        // add applied torques to 'fb' vector, including gyroscopic torque
        if (this->GetNoGyroTorque())
            this->variables_ptr->Get_fb().PasteSumVector((Xtorque)*factor, 3, 0);
        else
            this->variables_ptr->Get_fb().PasteSumVector((Xtorque - gyro) * factor, 3, 0);
    }

    void FrHydroBody::VariablesQbLoadSpeed() {
        // set current speed in 'qb', it can be used by the solver when working in incremental mode
        this->variables_ptr->Get_qb().PasteVector(GetCoord_dt().pos, 0, 0);
        this->variables_ptr->Get_qb().PasteVector(GetWvel_loc(), 3, 0);
    }

    void FrHydroBody::VariablesQbSetSpeed(double step) {
        chrono::ChCoordsys<> old_coord_dt = this->GetCoord_dt();

        // from 'qb' vector, sets body speed, and updates auxiliary data
        this->SetPos_dt(this->variables_ptr->Get_qb().ClipVector(0, 0));
        this->SetWvel_loc(this->variables_ptr->Get_qb().ClipVector(3, 0));

        // apply limits (if in speed clamping mode) to speeds.
        ClampSpeed();

        // compute auxiliary gyroscopic forces
        ComputeGyro();

        // Compute accel. by BDF (approximate by differentiation);
        if (step) {
            this->SetPos_dtdt((this->GetCoord_dt().pos - old_coord_dt.pos) / step);
            this->SetRot_dtdt((this->GetCoord_dt().rot - old_coord_dt.rot) / step);
        }
    }

    void FrHydroBody::VariablesQbIncrementPosition(double dt_step) {
        if (!this->IsActive())
            return;

        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

        chrono::ChVector<> newspeed = variables_ptr->Get_qb().ClipVector(0, 0);
        chrono::ChVector<> newwel = variables_ptr->Get_qb().ClipVector(3, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        this->SetPos(this->GetPos() + newspeed * dt_step);

        // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
        chrono::ChQuaternion<> mdeltarot;
        chrono::ChQuaternion<> moldrot = this->GetRot();
        chrono::ChVector<> newwel_abs = Amatrix * newwel;
        double mangle = newwel_abs.Length() * dt_step;
        newwel_abs.Normalize();
        mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
        chrono::ChQuaternion<> mnewrot = mdeltarot % moldrot;
        this->SetRot(mnewrot);
    }

}  // end namespace frydom
