//
// Created by frongere on 21/06/17.
//

#include "FrBody.h"
#include "FrNode.h"
#include "FrRotation.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "FrFrame.h"

namespace frydom {

    void FrBody::AddNode(std::shared_ptr<FrNode> node) {
        // Adding the node as a marker to the body
        AddMarker(node);
        // TODO: PUT Force related stuff here

    }

    std::shared_ptr<FrNode> FrBody::CreateNode() {

        // Creating the node and updating its position
        auto node = std::make_shared<FrNode>();
        node->SetBody(this);  // FIXME: a priori, c'est deja fait dans AddMarker lors de l'appele a AddNode... A retirer
        node->UpdateState();  // TODO: voir s'il est besoin d'appeler l'update...

        AddNode(node);
        return node;
    }

    std::shared_ptr<FrNode> FrBody::CreateNode(const chrono::ChVector<double> relpos) {

        auto node = CreateNode();

//        node->SetPos(relpos);  // TODO: Voir a utiliser ImposeRelPos tel que demande sur la liste chrono
        chrono::ChCoordsys<> coord;
        coord.pos = relpos;
        node->Impose_Rel_Coord(coord);
        node-> UpdateState();

        return node;
    }


    void FrBody::SetVisuMesh(std::shared_ptr<FrTriangleMeshConnected> mesh) {

        m_visu_mesh = mesh;

        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        AddAsset(shape);

    }

    void FrBody::SetVisuMesh(std::string obj_filename) {
        auto mesh=std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        SetVisuMesh(mesh);
    }












    /// REFACTORING ------------->>>>>>>>>>>>>>>


    #define DEFAULT_MAX_SPEED (float)10.
    #define DEFAULT_MAX_ROTATION_SPEED (float)180.*DEG2RAD


    _FrBodyBase::_FrBodyBase() : chrono::ChBodyAuxRef() {}


    FrBody_::FrBody_() {
        m_chronoBody = std::make_shared<_FrBodyBase>();
        m_chronoBody->SetMaxSpeed(DEFAULT_MAX_SPEED);
        m_chronoBody->SetMaxWvel(DEFAULT_MAX_ROTATION_SPEED);
    }

    void FrBody_::SetName(const char *name) {
        m_chronoBody->SetName(name);
    }

    void FrBody_::SetBodyFixed(bool state) {
        m_chronoBody->SetBodyFixed(state);
    }

    void FrBody_::Initialize() {

        // Initializing forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->Initialize();
        }

        // TODO : initialiser les logs

    }

    void FrBody_::StepFinalize() {
        // TODO
    }

    void FrBody_::SetSmoothContact() {
        auto materialSurface = std::make_shared<chrono::ChMaterialSurfaceSMC>();
        m_chronoBody->SetMaterialSurface(materialSurface);
        m_contactType = CONTACT_TYPE::SMOOTH_CONTACT;
    }

    void FrBody_::SetNonSmoothContact() {
        auto materialSurface = std::make_shared<chrono::ChMaterialSurfaceNSC>();
        m_chronoBody->SetMaterialSurface(materialSurface);
        m_contactType = CONTACT_TYPE::NONSMOOTH_CONTACT;
    }

    void FrBody_::SetContactMethod(CONTACT_TYPE contactType) {
        switch (contactType) {
            case CONTACT_TYPE::SMOOTH_CONTACT:
                SetSmoothContact();
                break;
            case CONTACT_TYPE::NONSMOOTH_CONTACT:
                SetNonSmoothContact();
                break;
        }
    }

    FrBody_::CONTACT_TYPE FrBody_::GetContactType() const {
        return m_contactType;
    }


    // Linear iterators

    FrBody_::ForceIter FrBody_::force_begin() {
        return m_externalForces.begin();
    }

    FrBody_::ConstForceIter FrBody_::force_begin() const {
        return m_externalForces.cbegin();
    }

    FrBody_::ForceIter FrBody_::force_end() {
        return m_externalForces.end();
    }

    FrBody_::ConstForceIter FrBody_::force_end() const {
        return m_externalForces.cend();
    }


    void FrBody_::AddMeshAsset(std::string obj_filename) {

        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        m_chronoBody->AddAsset(shape);

    }

    double FrBody_::GetMass() const {
        return m_chronoBody->GetMass();
    }

    Position FrBody_::GetCOGLocalPosition(FRAME_CONVENTION fc) const {  // OK
        auto cogPos = m_chronoBody->GetFrame_COG_to_REF().GetPos(); // In NWU

        auto frPos = internal::ChVectorToVector3d<Position>(cogPos);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(frPos);
        return frPos;
    }

    void FrBody_::SetCOGLocalPosition(double x, double y, double z, bool transportInertia, FRAME_CONVENTION fc) {

        auto cogFrame = chrono::ChFrame<double>();
        cogFrame.SetPos(internal::MakeNWUChVector(x, y, z, fc));  // TODO : regarder partout ou on utlise le SwapVectorFrameConvention... et voir si on ne peut pas remplacer par MakeNWUChvector...
        m_chronoBody->SetFrame_COG_to_REF(cogFrame);

        m_chronoBody->Update();  // To make auxref_to_abs up to date

        if (transportInertia) {  // FIXME : pas certain que ca fonctionne !!
            m_chronoBody->SetInertia(
                    m_chronoBody->GetInertia() +
                    internal::GetPointMassInertia(GetMass(), Position(-x, -y, -z))
                    );
        }
    }

    void FrBody_::SetCOGLocalPosition(const Position& position, bool transportInertia, FRAME_CONVENTION fc) {
        SetCOGLocalPosition(position[0], position[1], position[2], transportInertia, fc);
    }

    FrInertiaTensor_ FrBody_::GetInertiaParams() const {
        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        internal::ChInertia2Coeffs(m_chronoBody->GetInertia(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

        auto cogPos = GetCOGLocalPosition(NWU);

        return FrInertiaTensor_(GetMass(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame_(cogPos, FrRotation_(), NWU), NWU);
    }

    void FrBody_::SetInertiaParams(const FrInertiaTensor_& inertia) {

        m_chronoBody->SetMass(inertia.GetMass());

        SetCOGLocalPosition(inertia.GetCOGPosition(NWU), false, NWU);

        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        inertia.GetInertiaCoeffs(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

        m_chronoBody->SetInertiaXX(chrono::ChVector<double>(Ixx, Iyy, Izz));
        m_chronoBody->SetInertiaXY(chrono::ChVector<double>(Ixy, Ixz, Iyz));

    }

    void FrBody_::SetInertiaParams(double mass,
                          double Ixx, double Iyy, double Izz,
                          double Ixy, double Ixz, double Iyz,
                          const FrFrame_& coeffsFrame,
                          const Position& cogPosition,
                          FRAME_CONVENTION fc) {
        SetInertiaParams(FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, coeffsFrame, cogPosition, fc));
    }

    void FrBody_::SetInertiaParams(double mass,
                          double Ixx, double Iyy, double Izz,
                          double Ixy, double Ixz, double Iyz,
                          const FrFrame_& cogFrame,
                          FRAME_CONVENTION fc) {
        SetInertiaParams(FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, cogFrame, fc));
    }

    void FrBody_::SetCollide(bool isColliding) {
        m_chronoBody->SetCollide(isColliding);
    }

    void FrBody_::AddBoxShape(double xSize, double ySize, double zSize) {
        auto shape = std::make_shared<chrono::ChBoxShape>();
        shape->GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::AddCylinderShape(double radius, double height) {
        auto shape = std::make_shared<chrono::ChCylinderShape>();
        shape->GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
        shape->GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
        shape->GetCylinderGeometry().rad = radius;
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::AddSphereShape(double radius) {
        auto shape = std::make_shared<chrono::ChSphereShape>();
        shape->GetSphereGeometry().rad = radius;
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::ActivateSpeedLimits(bool activate) {
        m_chronoBody->SetLimitSpeed(activate);
    }

    void FrBody_::SetMaxSpeed(double maxSpeed_ms) {
        m_chronoBody->SetMaxSpeed((float)maxSpeed_ms);
    }

    void FrBody_::SetMaxRotationSpeed(double wMax_rads) {
        m_chronoBody->SetMaxWvel((float)wMax_rads);
    }

    void FrBody_::RemoveGravity(bool val) { // TODO : ajouter la force d'accumulation a l'initialisation --> cas ou le systeme n'a pas encore ete precise pour la gravite...

        if (val) {
            m_chronoBody->Accumulate_force(
                    GetMass() * m_chronoBody->TransformDirectionParentToLocal(chrono::ChVector<double>(0., 0., 9.81)),
                    chrono::VNULL,
                    true
                    );
            // TODO : aller chercher la gravite dans systeme !!!
        } else {
            m_chronoBody->Empty_forces_accumulators();
        }

    }

    void FrBody_::SetCOGAbsPosition(double x, double y, double z, FRAME_CONVENTION fc) {  // OK

        m_chronoBody->SetPos(internal::MakeNWUChVector(x, y, z, fc));
        m_chronoBody->Update();
    }

    void FrBody_::SetCOGAbsPosition(Position position, FRAME_CONVENTION fc) {  // OK
        SetCOGAbsPosition(position[0], position[1], position[2], fc);
    }

    void FrBody_::GetCOGAbsPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const {  // OK
        auto pos = m_chronoBody->GetPos(); // In NWU
        x = pos.x();
        y = pos.y();
        z = pos.z();

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NED frame convention
    }

    Position FrBody_::GetCOGAbsPosition(FRAME_CONVENTION fc) const {  // OK
        auto cogPos = m_chronoBody->GetPos(); // In NWU

        auto frPos = internal::ChVectorToVector3d<Position>(cogPos);  // In NWU

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(frPos);
        return frPos;
    }

    void FrBody_::SetAbsPosition(double x, double y, double z, FRAME_CONVENTION fc) {  // OK

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NWU

        chrono::ChFrame<double> auxFrame;
        auxFrame.SetPos(chrono::ChVector<double>(x, y, z));
        m_chronoBody->SetFrame_REF_to_abs(auxFrame);
    }

    void FrBody_::SetAbsPosition(const Position& position, FRAME_CONVENTION fc) {  // OK
        auto posTmp = position;
        SetAbsPosition(posTmp[0], posTmp[1], posTmp[2], fc);
    }

    Position FrBody_::GetAbsPosition(FRAME_CONVENTION fc) const {  // OK
        auto pos = m_chronoBody->GetFrame_REF_to_abs().GetPos();  // In NWU
        auto frPos = internal::ChVectorToVector3d<Position>(pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(frPos);  // Convert into NED
        return frPos;
    }

    void FrBody_::GetAbsPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const {  // OK
        auto pos = m_chronoBody->GetFrame_REF_to_abs().GetPos();
        x = pos[0];
        y = pos[1];
        z = pos[2];
        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NED
    }

    void FrBody_::GetAbsPosition(Position &position, FRAME_CONVENTION fc) const {  // OK
        position = GetAbsPosition(fc);
    }

    FrFrame_ FrBody_::GetAbsFrame() const {  // OK
        return internal::Ch2FrFrame(m_chronoBody->GetFrame_REF_to_abs());
    }

    Position FrBody_::GetAbsPositionOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {

        Position refFramePosition = GetAbsPosition(fc);

        Position pointPosition = Position(x, y, z);

        Position absPos; // It is mandatory to declare the vector appart from the calculation...--> Eigen
        absPos = refFramePosition + pointPosition;

        return absPos;
    }

    FrFrame_ FrBody_::GetOtherFrameRelativeTransform_WRT_ThisBody(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const {
        return GetAbsFrame().GetOtherFrameRelativeTransform_WRT_ThisFrame(otherFrame, fc);
    }

    FrRotation_ FrBody_::GetAbsRotation() const {
        return FrRotation_(GetAbsQuaternion());
    }

    FrQuaternion_ FrBody_::GetAbsQuaternion() const {
        return internal::Ch2FrQuaternion(m_chronoBody->GetFrame_REF_to_abs().GetRot());
    }

    void FrBody_::GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {
        GetAbsRotation().GetEulerAngles_RADIANS(phi, theta, psi, seq, fc);
    }

    void FrBody_::GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {
        GetAbsRotation().GetEulerAngles_DEGREES(phi, theta, psi, seq, fc);
    }

    void FrBody_::GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
        GetAbsRotation().GetCardanAngles_RADIANS(phi, theta, psi, fc);
    }

    void FrBody_::GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
        GetAbsRotation().GetCardanAngles_DEGREES(phi, theta, psi, fc);
    }

    void FrBody_::GetRotationAxisAngle(Direction &axis, double angle, FRAME_CONVENTION fc) const {
        GetAbsRotation().GetAxisAngle(axis, angle, fc);
    }

    double FrBody_::GetRoll_DEGREES(FRAME_CONVENTION fc) const {
        double phi, theta, psi;
        GetCardanAngles_DEGREES(phi, theta, psi, fc);
        return phi;
    }

    double FrBody_::GetPitch_DEGREES(FRAME_CONVENTION fc) const {
        double phi, theta, psi;
        GetCardanAngles_DEGREES(phi, theta, psi, fc);
        return theta;
    }

    double FrBody_::GetYaw_DEGREES(FRAME_CONVENTION fc) const {
        double phi, theta, psi;
        GetCardanAngles_DEGREES(phi, theta, psi, fc);
        return psi;
    }

    double FrBody_::GetRoll_RADIANS(FRAME_CONVENTION fc) const {
        double phi, theta, psi;
        GetCardanAngles_RADIANS(phi, theta, psi, fc);
        return phi;
    }

    double FrBody_::GetPitch_RADIANS(FRAME_CONVENTION fc) const {
        double phi, theta, psi;
        GetCardanAngles_RADIANS(phi, theta, psi, fc);
        return theta;
    }

    double FrBody_::GetYaw_RADIANS(FRAME_CONVENTION fc) const {
        double phi, theta, psi;
        GetCardanAngles_RADIANS(phi, theta, psi, fc);
        return psi;
    }

    void FrBody_::SetAbsRotation(const FrRotation_ &rotation, FRAME_CONVENTION fc) {
        m_chronoBody->SetRot(internal::Fr2ChQuaternion(rotation.GetQuaternion()));
    }

    void FrBody_::SetAbsRotation(const FrQuaternion_ &quaternion, FRAME_CONVENTION fc) {
        m_chronoBody->SetRot(internal::Fr2ChQuaternion(quaternion));
    }

    void
    FrBody_::SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
        if (IsNED(fc)) { // Convert into NWU
            theta = -theta;
            psi = -psi;
        }
        m_chronoBody->SetRot(internal::euler_to_quat(phi, theta, psi, seq, RAD));
    }

    void
    FrBody_::SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
        SetEulerAngles_RADIANS(phi*DEG2RAD, theta*DEG2RAD, psi*DEG2RAD, seq, fc);
    }

    void FrBody_::SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc) {
        SetEulerAngles_RADIANS(phi, theta, psi, CARDAN, fc);
    }

    void FrBody_::SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc) {
        SetEulerAngles_DEGREES(phi, theta, psi, CARDAN, fc);
    }

    void FrBody_::SetRotationAxisAngle(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) {
        SetAbsRotation(FrQuaternion_(axis, angleRAD, fc), fc);
    }

    void FrBody_::SetRoll_DEGREES(double roll, FRAME_CONVENTION fc) {
        SetCardanAngles_DEGREES(roll, 0., 0., fc);
    }

    double FrBody_::SetPitch_DEGREES(double pitch, FRAME_CONVENTION fc) {
        SetCardanAngles_DEGREES(0., pitch, 0., fc);
    }

    double FrBody_::SetYaw_DEGREES(double yaw, FRAME_CONVENTION fc) {
        SetCardanAngles_DEGREES(0., 0., yaw, fc);
    }

    double FrBody_::SetRoll_RADIANS(double roll, FRAME_CONVENTION fc) {
        SetCardanAngles_RADIANS(roll, 0., 0., fc);
    }

    double FrBody_::SetPitch_RADIANS(double pitch, FRAME_CONVENTION fc) {
        SetCardanAngles_RADIANS(0., pitch, 0., fc);
    }

    double FrBody_::SetYaw_RADIANS(double yaw, FRAME_CONVENTION fc) {
        SetCardanAngles_RADIANS(0., 0., yaw, fc);
    }

    void FrBody_::SetAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc) {
        // TODO

        if (IsNED(fc)) internal::SwapCoordinateConvention(vx, vy, vz);  // Convert into NWU
        auto localReferenceVelocity = chrono::ChVector<double>(vx, vy, vz);

        // Computing the resulting COG absolute velocity

        // Getting the absolute rotational velocity vector expressed in
        auto omega = m_chronoBody->GetWvel_par();

        auto GL = m_chronoBody->GetFrame_REF_to_abs().GetPos() - m_chronoBody->GetFrame_COG_to_abs().GetPos();

        m_chronoBody->SetPos_dt(localReferenceVelocity + GL.Cross(omega));

    }

    void FrBody_::SetAbsVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        SetAbsVelocity(velocity.GetVx(), velocity.GetVy(), velocity.GetVz(), fc);
    }

    Velocity FrBody_::GetAbsVelocity(FRAME_CONVENTION fc) const {
        Velocity velocity;
        GetAbsVelocity(velocity, fc);
        return velocity;
    }

    void FrBody_::GetAbsVelocity(Velocity &velocity, FRAME_CONVENTION fc) const {
        velocity = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
    }

    void FrBody_::GetAbsVelocity(double &vx, double &vy, double &vz, FRAME_CONVENTION fc) const {
        auto velocity = GetAbsVelocity(fc);
        vx = velocity.GetVx();
        vy = velocity.GetVy();
        vz = velocity.GetVz();
    }

    void FrBody_::SetLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(u, v, w);
        auto absVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->TransformDirectionLocalToParent(chrono::ChVector<double>(u, v, w)));
        SetAbsVelocity(absVel, NWU);
    }

    void FrBody_::SetLocalVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        SetLocalVelocity(velocity.GetVx(), velocity.GetVy(), velocity.GetVz(), fc);
    }

    Velocity FrBody_::GetLocalVelocity(FRAME_CONVENTION fc) const {
        Velocity relVel;
        GetLocalVelocity(relVel, fc);
        return relVel;
    }

    void FrBody_::GetLocalVelocity(Velocity &velocity, FRAME_CONVENTION fc) const {
        auto absVel = internal::Vector3dToChVector(GetAbsVelocity(NWU));
        velocity = internal::ChVectorToVector3d<Velocity>(m_chronoBody->TransformDirectionParentToLocal(absVel));
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
    }

    void FrBody_::GetLocalVelocity(double &u, double &v, double &w, FRAME_CONVENTION fc) const {
        auto relVel = GetLocalVelocity(fc);
        u = relVel.GetVx();
        v = relVel.GetVy();
        w = relVel.GetVz();
    }

    Velocity FrBody_::GetAbsVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NWU
        auto omega = m_chronoBody->GetWvel_par();
        auto PG = m_chronoBody->GetPos() - m_chronoBody->TransformDirectionLocalToParent(chrono::ChVector<double>(x, y, z));

        auto absvelTmp = m_chronoBody->GetPos_dt();
        absvelTmp += PG.Cross(omega);

        auto absVel = internal::ChVectorToVector3d<Velocity>(absvelTmp);
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(absVel);
        return absVel;
    }

    Velocity FrBody_::GetLocalVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        Velocity absVel = GetAbsVelocityOfLocalPoint(x, y, z, fc);
        return GetAbsQuaternion().Inverse().Rotate<Velocity>(absVel, fc);  // TODO : verifier
    }

    void FrBody_::SetCOGAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(vx, vy, vz); // Convert to NWU
        m_chronoBody->SetPos_dt(chrono::ChVector<double>(vx, vy, vz));
    }

    void FrBody_::SetCOGAbsVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);  // Convert to NWU
        m_chronoBody->SetPos_dt(internal::Vector3dToChVector(velocity));
    }

    Velocity FrBody_::GetCOGAbsVelocity(FRAME_CONVENTION fc) const {
        Velocity velocity;
        GetCOGAbsVelocity(velocity, fc);
        return velocity;
    }

    void FrBody_::GetCOGAbsVelocity(Velocity &velocity, FRAME_CONVENTION fc) const {
        velocity = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
    }

    void FrBody_::GetCOGAbsVelocity(double &vx, double &vy, double &vz, FRAME_CONVENTION fc) const {
        auto v = GetCOGAbsVelocity(fc);
        vx = v[0];
        vy = v[1];
        vz = v[2];
    }

    void FrBody_::SetCOGLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(u, v, w);

        m_chronoBody->SetPos_dt(m_chronoBody->TransformDirectionLocalToParent(chrono::ChVector<double>(u, v, w)));
    }

    void FrBody_::SetCOGLocalVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        SetCOGLocalVelocity(velocity.GetVx(), velocity.GetVy(), velocity.GetVz(), fc);
    }

    Velocity FrBody_::GetCOGLocalVelocity(FRAME_CONVENTION fc) const {
        Velocity relVel;
        GetCOGLocalVelocity(relVel, fc);
        return relVel;
    }

    void FrBody_::GetCOGLocalVelocity(Velocity &velocity, FRAME_CONVENTION fc) const {
        velocity = internal::ChVectorToVector3d<Velocity>(m_chronoBody->TransformDirectionParentToLocal(m_chronoBody->GetPos_dt()));
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
    }

    void FrBody_::GetCOGLocalVelocity(double &u, double &v, double &w, FRAME_CONVENTION fc) const {
        auto relVel = GetCOGLocalVelocity(fc);
        u = relVel.GetVx();
        v = relVel.GetVy();
        w = relVel.GetVz();
    }

    void FrBody_::SetAbsRotationalVelocity(double wx, double wy, double wz, FRAME_CONVENTION fc) {  // OK si le COG est sur le frame
        if (IsNED(fc)) internal::SwapCoordinateConvention(wx, wy, wz); // Convert into NWU
        m_chronoBody->SetWvel_par(chrono::ChVector<double>(wx, wy, wz));
    }

    void FrBody_::SetAbsRotationalVelocity(const RotationalVelocity &omega, FRAME_CONVENTION fc) {
        SetAbsRotationalVelocity(omega.GetWx(), omega.GetWy(), omega.GetWz(), fc);
    }

    RotationalVelocity FrBody_::GetAbsRotationalVelocity(FRAME_CONVENTION fc) const {
        RotationalVelocity omega;
        GetAbsRotationalVelocity(omega, fc);
        return omega;
    }

    void FrBody_::GetAbsRotationalVelocity(RotationalVelocity &omega, FRAME_CONVENTION fc) const {
        omega = internal::ChVectorToVector3d<RotationalVelocity>(m_chronoBody->GetWvel_par());
        if (IsNED(fc)) internal::SwapFrameConvention<RotationalVelocity>(omega);
    }

    void FrBody_::GetAbsRotationalVelocity(double &wx, double &wy, double &wz, FRAME_CONVENTION fc) const {
        auto omega = GetAbsRotationalVelocity(fc);
        wx = omega.GetWx();
        wy = omega.GetWy();
        wz = omega.GetWz();
    }


    // TODO : Utiliser les methodes PointSpeedLocalToParent etc... !!!!

    void FrBody_::SetAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(ax, ay, az);

        auto GO = -internal::Vector3dToChVector(GetCOGLocalPosition(NWU));

        auto cogAbsAcc = chrono::ChVector<double>(ax, ay, az) // local frame absolute acceleration
                - ((m_chronoBody->coord_dtdt.rot % chrono::ChQuaternion<double>(0, GO) % m_chronoBody->coord.rot.GetConjugate()).GetVector() * 2)
                - ((m_chronoBody->coord_dt.rot % chrono::ChQuaternion<double>(0, GO) % m_chronoBody->coord_dt.rot.GetConjugate()).GetVector() * 2);

        m_chronoBody->SetPos_dtdt(cogAbsAcc);
    }

    void FrBody_::SetAbsAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc) {
        // TODO
    }

    Acceleration FrBody_::GetAbsAcceleration(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::GetAbsAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::GetAbsAcceleration(double &ax, double &ay, double &az, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::SetLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::SetLocalAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc) {
        // TODO
    }

    Acceleration FrBody_::GetLocalAcceleration(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::GetLocalAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::GetLocalAcceleration(double &up, double &vp, double &wp, FRAME_CONVENTION fc) const {
        // TODO
    }

    Acceleration FrBody_::GetAbsAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        // TODO
    }

    Acceleration FrBody_::GetLocalAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        // TODO
    }




}  // end namespace frydom