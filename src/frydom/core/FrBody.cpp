//
// Created by frongere on 21/06/17.
//


#include "FrBody.h"
#include "FrNode.h"
#include "FrRotation.h"
#include "FrMatrix.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "FrFrame.h"

#include "FrForce.h"

#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/environment/waves/FrFreeSurface.h"

#include <chrono/physics/ChLinkMotorLinearSpeed.h>  // FIXME : a retirer

#include "FrFunction.h"
#include "FrException.h"


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
    #define DEFAULT_MAX_ROTATION_SPEED (float)(180.*DEG2RAD)

    namespace internal {

        _FrBodyBase::_FrBodyBase(FrBody_ *body) : chrono::ChBodyAuxRef(), m_frydomBody(body) {}

        void _FrBodyBase::SetupInitial() {
            m_frydomBody->Initialize();
        }

        void _FrBodyBase::Update(bool update_assets) {
            chrono::ChBodyAuxRef::Update(update_assets);
            m_frydomBody->Update();
        }

    }  // end namespace internal

    FrBody_::FrBody_() {
        m_chronoBody = std::make_shared<internal::_FrBodyBase>(this);
        m_chronoBody->SetMaxSpeed(DEFAULT_MAX_SPEED);
        m_chronoBody->SetMaxWvel(DEFAULT_MAX_ROTATION_SPEED);
    }

    FrOffshoreSystem_* FrBody_::GetSystem(){
        return m_system;
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

    void FrBody_::Update() {
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
        AddMeshAsset(mesh);
    }

    void FrBody_::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrBody_::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        m_chronoBody->AddAsset(colorAsset);
    }




    void FrBody_::ConstainDOF(bool cx, bool cy, bool cz, bool crx, bool cry, bool crz) {

//        auto link = std::make_shared<>()

    }

    void FrBody_::ConstrainInVx(double Vx) {

        auto motor = std::make_shared<chrono::ChLinkMotorLinearSpeed>();

        auto frame = chrono::ChFrame<double>();


        motor->Initialize(m_chronoBody, GetSystem()->GetEnvironment()->GetFreeSurface()->m_body->m_chronoBody, frame);

        m_system->AddLink(motor);


        auto rampConst = std::make_shared<FrFunction>(Vx);

        motor->SetSpeedFunction(rampConst);



    }






    double FrBody_::GetMass() const {
        return m_chronoBody->GetMass();
    }

    Position FrBody_::GetCOGLocalPosition(FRAME_CONVENTION fc) const {  
        auto cogPos = m_chronoBody->GetFrame_COG_to_REF().GetPos(); // In NWU

        auto frPos = internal::ChVectorToVector3d<Position>(cogPos);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(frPos);
        return frPos;
    }

    void FrBody_::SetCOGLocalPosition(double x, double y, double z, bool transportInertia, FRAME_CONVENTION fc) {

//        auto cogFrame = chrono::ChFrame<double>();
//        cogFrame.SetPos(internal::MakeNWUChVector(x, y, z, fc));  // TODO : regarder partout ou on utlise le SwapVectorFrameConvention... et voir si on ne peut pas remplacer par MakeNWUChvector...
//        m_chronoBody->SetFrame_COG_to_REF(cogFrame);
//
//        m_chronoBody->Update(false);  // To make auxref_to_abs up to date

//        if (transportInertia) {  // FIXME : pas certain que ca fonctionne !!
//            m_chronoBody->SetInertia(
//                    m_chronoBody->GetInertia() +
//                    internal::GetPointMassInertia(GetMass(), Position(-x, -y, -z))
//                    );
//        }
    }

    void FrBody_::SetCOGLocalPosition(const Position& position, bool transportInertia, FRAME_CONVENTION fc) {
        SetCOGLocalPosition(position[0], position[1], position[2], transportInertia, fc);
    }

    FrInertiaTensor_ FrBody_::GetInertiaParams() const {
        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        SplitMatrix33IntoCoeffs(internal::ChMatrix33ToMatrix33(m_chronoBody->GetInertia()),
                Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

        auto cogPos = GetCOGLocalPosition(NWU);

        return {GetMass(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame_(cogPos, FrRotation_(), NWU), NWU};
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
        // TODO : this method should not be used in production !!
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

    void FrBody_::AddExternalForce(std::shared_ptr<frydom::FrForce_> force) {
        m_chronoBody->AddForce(force->GetChronoForce());  // FrBody_ is a friend class of FrForce_

        force->m_body = this;
        m_externalForces.push_back(force);
    }

    void FrBody_::RemoveExternalForce(std::shared_ptr<FrForce_> force) {
        m_chronoBody->RemoveForce(force->GetChronoForce());

        m_externalForces.erase(
                std::find<std::vector<std::shared_ptr<FrForce_>>::iterator>(m_externalForces.begin(), m_externalForces.end(), force));

        force->m_body = nullptr;
    }

    void FrBody_::RemoveAllForces() {
        m_chronoBody->RemoveAllForces();
        for (auto forceIter=force_begin(); forceIter!=force_end(); forceIter++) {
            (*forceIter)->m_body = nullptr;
        }
        m_externalForces.clear();
    }



    // Nodes

    std::shared_ptr<FrNode_> FrBody_::NewNode(const frydom::FrFrame_ &localFrame) {
        return std::make_shared<FrNode_>(this, localFrame);
    }

    std::shared_ptr<FrNode_> FrBody_::NewNode(const frydom::Position &localPosition) {
        return std::make_shared<FrNode_>(this, localPosition);
    }

    std::shared_ptr<FrNode_> FrBody_::NewNode(double x, double y, double z) {
        return std::make_shared<FrNode_>(this, Position(x, y, z));
    }


//    template <class Vector>
//    Vector FrBody_::ProjectLocalOnAbs(const Vector &vector) {
//        return internal::ChVectorToVector3d<Vector>(
//                m_chronoBody->TransformDirectionLocalToParent(
//                internal::Vector3dToChVector(vector)));
//    }







    void FrBody_::SetCOGAbsPosition(double x, double y, double z, FRAME_CONVENTION fc) {  

//        m_chronoBody->SetPos(internal::MakeNWUChVector(x, y, z, fc));
//        m_chronoBody->Update(false); // To make the auxref up to date
    }

    void FrBody_::SetCOGAbsPosition(Position position, FRAME_CONVENTION fc) {  
        SetCOGAbsPosition(position[0], position[1], position[2], fc);
    }

    void FrBody_::GetCOGAbsPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const {  
        auto pos = m_chronoBody->GetPos(); // In NWU
        x = pos.x();
        y = pos.y();
        z = pos.z();

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NED frame convention
    }

    Position FrBody_::GetCOGAbsPosition(FRAME_CONVENTION fc) const {  
        auto cogPos = m_chronoBody->GetPos(); // In NWU

        auto frPos = internal::ChVectorToVector3d<Position>(cogPos);  // In NWU

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(frPos);
        return frPos;
    }

    void FrBody_::SetAbsPosition(double x, double y, double z, FRAME_CONVENTION fc) {  
        // Il faut que le COG et les markers soient bouges en consequence !!
        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NWU

        chrono::ChFrame<double> auxFrame;
        auxFrame.SetPos(chrono::ChVector<double>(x, y, z));
        m_chronoBody->SetFrame_REF_to_abs(auxFrame);
        m_chronoBody->UpdateMarkers(m_chronoBody->GetChTime()); // To update markers too // TODO : voir si suffisant...
    }

    void FrBody_::SetAbsPosition(const Position& position, FRAME_CONVENTION fc) {  
        auto posTmp = position;
        SetAbsPosition(posTmp[0], posTmp[1], posTmp[2], fc);
    }

    Position FrBody_::GetAbsPosition(FRAME_CONVENTION fc) const {  
        double x, y, z;
        GetAbsPosition(x, y, z, fc);
        return {x, y, z};
    }

    void FrBody_::GetAbsPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const {  
        auto pos = m_chronoBody->GetFrame_REF_to_abs().GetPos();
        x = pos[0];
        y = pos[1];
        z = pos[2];
        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);  // Convert into NED
    }

    void FrBody_::GetAbsPosition(Position &position, FRAME_CONVENTION fc) const {  
        position = GetAbsPosition(fc);
    }

    FrFrame_ FrBody_::GetAbsFrame() const {  
        return internal::Ch2FrFrame(m_chronoBody->GetFrame_REF_to_abs());
    }

    Position FrBody_::GetAbsPositionOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {  
        return GetAbsPositionOfLocalPoint(Position(x, y, z), fc);
    }

    Position FrBody_::GetAbsPositionOfLocalPoint(const Position& localPos, FRAME_CONVENTION fc) const {
        return GetAbsPosition(fc) + ProjectBodyVectorInAbsCoords(localPos, fc);
    }

    Position FrBody_::GetLocalPositionOfAbsPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        return GetLocalPositionOfAbsPoint(Position(x, y, z), fc);
    }

    Position FrBody_::GetLocalPositionOfAbsPoint(const Position& absPos, FRAME_CONVENTION fc) const {
        return ProjectAbsVectorInBodyCoords<Position>(absPos - GetAbsPosition(fc), fc);
    }

    FrFrame_ FrBody_::GetOtherFrameRelativeTransform_WRT_ThisBody(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const {
        return GetAbsFrame().GetInverse() * otherFrame;  // TODO : verifier !!
    }

    FrRotation_ FrBody_::GetAbsRotation() const {  
        return FrRotation_(GetAbsQuaternion());
    }

    FrQuaternion_ FrBody_::GetAbsQuaternion() const {  
        return internal::Ch2FrQuaternion(m_chronoBody->GetRot());
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

    void FrBody_::GetRotationAxisAngle(Direction &axis, double angleRAD, FRAME_CONVENTION fc) const {
        GetAbsRotation().GetAxisAngle(axis, angleRAD, fc);
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

    void FrBody_::SetAbsRotation(const FrRotation_ &rotation) {
        SetAbsRotation(rotation.GetQuaternion());
    }

    void FrBody_::SetAbsRotation(const FrQuaternion_ &quaternion) {
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
        SetAbsRotation(FrQuaternion_(axis, angleRAD, fc));
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

        if (IsNED(fc)) internal::SwapCoordinateConvention(vx, vy, vz);

        m_chronoBody->SetPos_dt(
                chrono::ChVector<double>(vx, vy, vz) -
                       (m_chronoBody->coord_dt.rot *
                        chrono::ChQuaternion<double>(0., m_chronoBody->GetFrame_REF_to_COG().GetPos()) *
                        m_chronoBody->coord_dt.rot.GetConjugate()).GetVector() * 2
                );
        m_chronoBody->UpdateMarkers(m_chronoBody->GetChTime());
    }

    void FrBody_::SetAbsVelocity(const Velocity &absVel, FRAME_CONVENTION fc) {
        SetAbsVelocity(absVel.GetVx(), absVel.GetVy(), absVel.GetVz(), fc);
    }

    Velocity FrBody_::GetAbsVelocity(FRAME_CONVENTION fc) const {  
        Velocity velocity;
        GetAbsVelocity(velocity, fc);
        return velocity;
    }

    void FrBody_::GetAbsVelocity(Velocity &absVel, FRAME_CONVENTION fc) const {
        absVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(absVel);
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

        // TODO : utiliser l'API
        auto absVel = internal::ChVectorToVector3d<Velocity>(
                m_chronoBody->PointSpeedLocalToParent(chrono::ChVector<double>(x, y, z) - m_chronoBody->GetFrame_COG_to_REF().GetPos()));

        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(absVel);

        return absVel;
    }

    Velocity FrBody_::GetAbsVelocityOfLocalPoint(const Position& locaPos, FRAME_CONVENTION fc) const {
        return GetAbsVelocityOfLocalPoint(locaPos.GetX(), locaPos.GetY(), locaPos.GetZ(), fc);
    }

    Velocity FrBody_::GetLocalVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        Velocity absVel = GetAbsVelocityOfLocalPoint(x, y, z, fc);
        return GetAbsQuaternion().Inverse().Rotate<Velocity>(absVel, fc);  // TODO : verifier
    }

    Velocity FrBody_::GetAbsRelVelocityInStreamAtAbsPoint(const Position &absPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const {

        // Gettting the absolute velocity of the body at this absolute point in space (body motion field...)
        // Note that everything is done in fc frame convention
        Velocity absVel = GetAbsVelocityOfLocalPoint(absPos, fc);

        Velocity absRelVel;
        switch (ft) {

            case WATER:
                absRelVel = m_system->GetEnvironment()->GetCurrent()->GetAbsRelativeVelocity(absPos, absVel, fc);
                break;

            case AIR:
                absRelVel = m_system->GetEnvironment()->GetWind()->GetAbsRelativeVelocity(absPos, absVel, fc);
                break;

            default:
                throw FrException("Fluid is not known...");
        }

        return absRelVel;
    }

    Velocity FrBody_::GetAbsRelVelocityInStreamAtCOG(FLUID_TYPE ft, FRAME_CONVENTION fc) const {
        return GetAbsRelVelocityInStreamAtAbsPoint(GetCOGAbsPosition(fc), ft, fc);
    }

    Velocity FrBody_::GetLocalRelVelocityInStreamAtCOG(FLUID_TYPE ft, FRAME_CONVENTION fc) const {
        return ProjectAbsVectorInBodyCoords<Velocity>(GetAbsRelVelocityInStreamAtCOG(ft, fc), fc);
    }

    Velocity FrBody_::GetAbsRelVelocityInStreamAtLocalPoint(const Position& localPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const {
        return GetAbsRelVelocityInStreamAtAbsPoint(GetAbsPositionOfLocalPoint(localPos, fc), ft, fc);
    }

    Velocity FrBody_::GetLocalRelVelocityInStreamAtLocalPoint(const Position& localPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const {
        return ProjectAbsVectorInBodyCoords<Velocity>(GetAbsRelVelocityInStreamAtLocalPoint(localPos, ft, fc), fc);
    }

    Velocity FrBody_::GetLocalRelVelocityInStreamAtAbsPoint(const Position& absPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const {
        return ProjectAbsVectorInBodyCoords<Velocity>(GetAbsRelVelocityInStreamAtAbsPoint(absPos, ft, fc), fc);
    }

    void FrBody_::SetCOGAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc) {  
        if (IsNED(fc)) internal::SwapCoordinateConvention(vx, vy, vz); // Convert to NWU
        m_chronoBody->SetPos_dt(chrono::ChVector<double>(vx, vy, vz));
    }

    void FrBody_::SetCOGAbsVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {  
        SetCOGAbsVelocity(velocity.GetVx(), velocity.GetVy(), velocity.GetVz(), fc);
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

    void FrBody_::SetAbsRotationalVelocity(double wx, double wy, double wz, FRAME_CONVENTION fc) {  
        if (IsNED(fc)) internal::SwapCoordinateConvention(wx, wy, wz); // Convert into NWU
        m_chronoBody->SetWvel_par(chrono::ChVector<double>(wx, wy, wz));
    }

    void FrBody_::SetAbsRotationalVelocity(const AngularVelocity &omega, FRAME_CONVENTION fc) {
        SetAbsRotationalVelocity(omega.GetWx(), omega.GetWy(), omega.GetWz(), fc);
    }

    AngularVelocity FrBody_::GetAbsRotationalVelocity(FRAME_CONVENTION fc) const {
        AngularVelocity omega;
        GetAbsRotationalVelocity(omega, fc);
        return omega;
    }

    void FrBody_::GetAbsRotationalVelocity(AngularVelocity &omega, FRAME_CONVENTION fc) const {
        omega = internal::ChVectorToVector3d<AngularVelocity>(m_chronoBody->GetWvel_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(omega);
    }

    void FrBody_::GetAbsRotationalVelocity(double &wx, double &wy, double &wz, FRAME_CONVENTION fc) const {  
        auto omega = GetAbsRotationalVelocity(fc);
        wx = omega.GetWx();
        wy = omega.GetWy();
        wz = omega.GetWz();
    }

    void FrBody_::SetLocalRotationalVelocity(double p, double q, double r, FRAME_CONVENTION fc) {  
        if (IsNED(fc)) internal::SwapCoordinateConvention(p, q, r);
        auto absRotVel = m_chronoBody->TransformDirectionLocalToParent(chrono::ChVector<double>(p, q, r));
        SetAbsRotationalVelocity(absRotVel[0], absRotVel[1], absRotVel[2], NWU);
    }

    void FrBody_::SetLocalRotationalVelocity(const AngularVelocity &omega, FRAME_CONVENTION fc) {
        SetLocalRotationalVelocity(omega.GetWx(), omega.GetWy(), omega.GetWz(), fc);
    }

    AngularVelocity FrBody_::GetLocalRotationalVelocity(FRAME_CONVENTION fc) const {
        AngularVelocity rotVel;
        GetLocalRotationalVelocity(rotVel, fc);
        return rotVel;
    }

    void FrBody_::GetLocalRotationalVelocity(AngularVelocity &omega, FRAME_CONVENTION fc) const {
        omega = internal::ChVectorToVector3d<AngularVelocity>(m_chronoBody->GetWvel_loc());  // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(omega);
    }

    void FrBody_::GetLocalRotationalVelocity(double &p, double &q, double &r, FRAME_CONVENTION fc) const {  
        auto rotVel = GetLocalRotationalVelocity(fc);
        p = rotVel.GetWx();
        q = rotVel.GetWy();
        r = rotVel.GetWz();
    }

    void FrBody_::SetAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc) {  
        if (IsNED(fc)) internal::SwapCoordinateConvention(ax, ay, az);

        auto GP = m_chronoBody->GetFrame_REF_to_COG().GetPos();

        m_chronoBody->SetPos_dtdt(
                chrono::ChVector<double>(ax, ay, az) -
                        (m_chronoBody->coord_dtdt.rot * chrono::ChQuaternion<double>(0., GP) * m_chronoBody->coord.rot.GetConjugate()).GetVector() * 2 -
                        (m_chronoBody->coord_dt.rot * chrono::ChQuaternion<double>(0., GP) * m_chronoBody->coord_dt.rot.GetConjugate()).GetVector() * 2
                );
    }

    void FrBody_::SetAbsAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc) {  
        SetAbsAcceleration(acceleration.GetAccX(), acceleration.GetAccY(), acceleration.GetAccZ(), fc);
    }

    Acceleration FrBody_::GetAbsAcceleration(FRAME_CONVENTION fc) const {  
        Acceleration absAcc;
        GetAbsAcceleration(absAcc, fc);
        return absAcc;
    }

    void FrBody_::GetAbsAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) const {  
        auto GP = m_chronoBody->GetFrame_REF_to_COG().GetPos();
        acceleration = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->PointAccelerationLocalToParent(GP));  // In NWU

        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(acceleration);
    }

    void FrBody_::GetAbsAcceleration(double &ax, double &ay, double &az, FRAME_CONVENTION fc) const {  
        auto absAcc = GetAbsAcceleration(fc);
        ax = absAcc.GetAccX();
        ay = absAcc.GetAccY();
        az = absAcc.GetAccZ();
    }

    void FrBody_::SetLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc) {  
        if (IsNED(fc)) internal::SwapCoordinateConvention(up, vp, wp);
        auto absAcc = m_chronoBody->TransformDirectionLocalToParent(chrono::ChVector<double>(up, vp, wp));
        SetAbsAcceleration(absAcc[0], absAcc[1], absAcc[2], NWU);
    }

    void FrBody_::SetLocalAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc) {  
        SetLocalAcceleration(acceleration.GetAccX(), acceleration.GetAccY(), acceleration.GetAccZ(), fc);
    }

    Acceleration FrBody_::GetLocalAcceleration(FRAME_CONVENTION fc) const {  
        Acceleration localAcc;
        GetLocalAcceleration(localAcc, fc);
        return localAcc;
    }

    void FrBody_::GetLocalAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) const {  
        auto absAcc = GetAbsAcceleration(NWU);
        acceleration = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->TransformDirectionParentToLocal(internal::Vector3dToChVector(absAcc)));
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(acceleration);
    }

    void FrBody_::GetLocalAcceleration(double &up, double &vp, double &wp, FRAME_CONVENTION fc) const {  
        auto localAcc = GetLocalAcceleration(fc);
        up = localAcc.GetAccX();
        vp = localAcc.GetAccY();
        wp = localAcc.GetAccZ();
    }

    Acceleration FrBody_::GetAbsAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);
        auto GP = chrono::ChVector<double>(x, y, z) - m_chronoBody->GetFrame_COG_to_REF().GetPos();
        auto absAcc = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->PointAccelerationLocalToParent(GP));
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(absAcc);
        return absAcc;
    }

    Acceleration FrBody_::GetLocalAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        auto absAcc = GetAbsAccelerationOfLocalPoint(x, y, z, fc); // In NWU
        return GetAbsQuaternion().Inverse().Rotate<Acceleration>(absAcc, fc);  // TODO : verifier
    }

    void FrBody_::SetCOGAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(ax, ay, az);
        m_chronoBody->coord_dtdt.pos = chrono::ChVector<double>(ax, ay, az);
    }

    void FrBody_::SetCOGAbsAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc) {
        SetCOGAbsAcceleration(acceleration.GetAccX(), acceleration.GetAccY(), acceleration.GetAccZ(), fc);
    }

    Acceleration FrBody_::GetCOGAbsAcceleration(FRAME_CONVENTION fc) const {
        Acceleration absAcc;
        GetCOGAbsAcceleration(absAcc, fc);
        return absAcc;
    }

    void FrBody_::GetCOGAbsAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) const {
        acceleration = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->coord_dtdt.pos);
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(acceleration);
    }

    void FrBody_::GetCOGAbsAcceleration(double &ax, double &ay, double &az, FRAME_CONVENTION fc) const {
        auto absAcc = GetCOGAbsAcceleration(fc);
        ax = absAcc.GetAccX();
        ay = absAcc.GetAccY();
        az = absAcc.GetAccZ();
    }

    void FrBody_::SetCOGLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(up, vp, wp);
        auto absAcc = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->TransformDirectionLocalToParent(chrono::ChVector<double>(up, vp, wp)));
        SetCOGAbsAcceleration(absAcc, NWU);
    }

    void FrBody_::SetCOGLocalAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc) {
        SetCOGLocalAcceleration(acceleration.GetAccX(), acceleration.GetAccY(), acceleration.GetAccZ(), fc);
    }

    Acceleration FrBody_::GetCOGLocalAcceleration(FRAME_CONVENTION fc) const {
        Acceleration localAcc;
        GetCOGLocalAcceleration(localAcc, fc);
        return localAcc;
    }

    void FrBody_::GetCOGLocalAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) const {
        auto absAcc = internal::Vector3dToChVector(GetCOGAbsAcceleration(NWU));
        acceleration = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->TransformDirectionParentToLocal(absAcc));
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(acceleration);
    }

    void FrBody_::GetCOGLocalAcceleration(double &up, double &vp, double &wp, FRAME_CONVENTION fc) const {
        auto localAcc = GetCOGLocalAcceleration(fc);
        up = localAcc.GetAccX();
        vp = localAcc.GetAccY();
        wp = localAcc.GetAccZ();
    }

    void FrBody_::SetAbsRotationalAcceleration(double wxp, double wyp, double wzp, FRAME_CONVENTION fc) {
        if (IsNED(fc)) internal::SwapCoordinateConvention(wxp, wyp, wzp);
        auto tmp = chrono::ChVector<double>(wxp, wyp, wzp);
        m_chronoBody->SetWacc_par(tmp);  // FIXME Chrono:: mettre en const tmp dans Chrono...
    }

    void FrBody_::SetAbsRotationalAcceleration(const AngularAcceleration &omegap, FRAME_CONVENTION fc) {
        SetAbsRotationalAcceleration(omegap.GetWxp(), omegap.GetWyp(), omegap.GetWzp(), fc);
    }

    AngularAcceleration FrBody_::GetAbsRotationalAcceleration(FRAME_CONVENTION fc) const {
        AngularAcceleration wAcc;
        GetAbsRotationalAcceleration(wAcc, fc);
        return wAcc;
    }

    void FrBody_::GetAbsRotationalAcceleration(AngularAcceleration &omegap, FRAME_CONVENTION fc) const {
        omegap = internal::ChVectorToVector3d<AngularAcceleration>(m_chronoBody->GetWacc_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(omegap);
    }

    void FrBody_::GetAbsRotationalAcceleration(double &wxp, double &wyp, double &wzp, FRAME_CONVENTION fc) const {
        auto wAcc = GetAbsRotationalAcceleration(fc);
        wxp = wAcc.GetWxp();
        wyp = wAcc.GetWyp();
        wzp = wAcc.GetWzp();
    }








    ///// NOUVELLES METHODES non implementees...


    void FrBody_::SetMass(double mass) {
        m_chronoBody->SetMass(mass);
    }

    void FrBody_::SetCOG(const Position& bodyPos, FRAME_CONVENTION fc) {
        FrFrame_ cogFrame;
        cogFrame.SetPosition(bodyPos, fc);
        m_chronoBody->SetFrame_COG_to_REF(internal::Fr2ChFrame(cogFrame));
    }

    Position FrBody_::GetCOG(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_COG_to_REF().GetPos()); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    Position FrBody_::GetPosition(FRAME_CONVENTION fc) const {
        return internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_REF_to_abs().GetPos());
    }

    void FrBody_::SetPosition(const Position &worldPos, FRAME_CONVENTION fc) {
        auto bodyFrame = GetFrame();
        bodyFrame.SetPosition(worldPos, fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::Fr2ChFrame(bodyFrame));
        // Traiter update
    }

    FrRotation_ FrBody_::GetRotation() const {
        return FrRotation_(GetQuaternion());
    }

    void FrBody_::SetRotation(const FrRotation_ &rotation) {
        SetRotation(rotation.GetQuaternion());
    }

    FrQuaternion_ FrBody_::GetQuaternion() const {
        return internal::Ch2FrQuaternion(m_chronoBody->GetRot());
    }

    void FrBody_::SetRotation(const FrQuaternion_ &quaternion) {
        m_chronoBody->SetRot(internal::Fr2ChQuaternion(quaternion));
    }

    FrFrame_ FrBody_::GetFrame() const {
        FrFrame_ bodyRefFrame;
        bodyRefFrame.SetPosition(GetPosition(NWU), NWU);
        bodyRefFrame.SetRotation(GetQuaternion());
        return bodyRefFrame;
    }

    void FrBody_::SetFrame(const FrFrame_ &worldFrame) {
        SetPosition(worldFrame.GetPosition(NWU), NWU);
        SetRotation(worldFrame.GetQuaternion());
    }

    Position FrBody_::GetPointPositionInWorld(const Position &bodyPos, FRAME_CONVENTION fc) const {
        return GetPosition(fc) + ProjectVectorInWorld<Position>(bodyPos, fc);
    }

    Position FrBody_::GetPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Position>(worldPos - GetPosition(fc), fc);
    }

    Position FrBody_::GetCOGPositionInWorld(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetPos());
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    void FrBody_::SetPointPosition(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc) {
        _SetPointPosition(bodyPoint, BODY, worldPos, WORLD, fc);
    }

    void FrBody_::SetCOGPosition(const Position &worldPos, FRAME_CONVENTION fc) {
        _SetPointPosition(GetCOG(fc), BODY, worldPos, WORLD, fc);
    }

    void FrBody_::TranslateInWorld(const Position &worldTranslation, FRAME_CONVENTION fc) {
        _Translate(worldTranslation, WORLD, fc);
    }

    void FrBody_::TranslateInBody(const Position &bodyTranslation, FRAME_CONVENTION fc) {
        _Translate(bodyTranslation, BODY, fc);
    }

    void FrBody_::_Translate(const Position &translation, FRAME translationFrame, FRAME_CONVENTION fc) {
        auto refFrame = GetFrame();

        if (translationFrame == BODY) {
            refFrame.SetPosition(refFrame.GetPosition(fc) + ProjectVectorInWorld<Position>(translation, fc), fc);
        } else {
            refFrame.SetPosition(refFrame.GetPosition(fc) + translation, fc);
        }

        m_chronoBody->SetFrame_REF_to_abs(internal::Fr2ChFrame(refFrame));
    }

//    void FrBody_::Rotate(const FrRotation_ &relRotation) {
//
//    }
//
//    void FrBody_::Rotate(const FrQuaternion_ &relQuaternion) {
//
//    }
//
//    void FrBody_::_SetVelocityAtPoint(const Position& point, FRAME pointFrame, FRAME pointFrameExpr,
//                                      const Velocity& vel, FRAME velFrame,
//                                      const AngularVelocity& angVel, FRAME angVelFrame, FRAME_CONVENTION fc) {
//
//        if (velFrame == BODY) {
//
//        }
//    }
//
//    void FrBody_::SetVelocityInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::SetVelocityInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
//
//    }
//
    Velocity FrBody_::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity bodyVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(bodyVel);
        return bodyVel;
    }

    Velocity FrBody_::GetVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetVelocityInWorld(fc), fc);
    }

    void FrBody_::SetCOGVelocityInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
        auto worldVelTmp = worldVel;
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(worldVelTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldVelTmp);
        m_chronoBody->SetCoord_dt(coord);
    }

    void FrBody_::SetCOGVelocityInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
        SetCOGVelocityInWorld(ProjectVectorInWorld(bodyVel, fc), fc);
    }

    Velocity FrBody_::GetCOGVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity cogVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetCoord_dt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(cogVel);
        return cogVel;
    }

    Velocity FrBody_::GetCOGVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetCOGVelocityInWorld(fc), fc);
    }

//    void FrBody_::SetAccelerationInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::SetAccelerationInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::GetAccelerationInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::GetAccelerationInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
//
//    }

    void FrBody_::SetCOGAccelerationInWorld(const Acceleration &worldAcc, FRAME_CONVENTION fc) {
        auto worldAccTmp = worldAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(worldAccTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldAccTmp);
        m_chronoBody->SetCoord_dtdt(coord);
    }

    void FrBody_::SetCOGAccelerationInBody(const Acceleration &bodyAcc, FRAME_CONVENTION fc) {
        SetCOGAccelerationInWorld(ProjectVectorInWorld<Acceleration>(bodyAcc, fc), fc);
    }

    Acceleration FrBody_::GetCOGAccelerationInWorld(FRAME_CONVENTION fc) const {
        Acceleration cogAcc = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->GetCoord_dtdt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(cogAcc);
        return cogAcc;
    }

    Acceleration FrBody_::GetCOGAccelerationInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Acceleration>(GetCOGAccelerationInWorld(fc), fc);
    }

    void FrBody_::SetAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        auto worldAngVelTmp = worldAngVel;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngVelTmp);
        m_chronoBody->SetWvel_par(internal::Vector3dToChVector(worldAngVelTmp));
    }

    void FrBody_::SetAngularVelocityInBody(const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        SetAngularVelocityInWorld(ProjectVectorInWorld(bodyAngVel, fc), fc);
    }

    AngularVelocity FrBody_::GetAngularVelocityInWorld(FRAME_CONVENTION fc) const {
        AngularVelocity angVel = internal::ChVectorToVector3d<AngularVelocity>(m_chronoBody->GetWvel_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(angVel);
        return angVel;
    }

    AngularVelocity FrBody_::GetAngularVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<AngularVelocity>(GetAngularVelocityInWorld(fc), fc);
    }

    void FrBody_::SetAngularAccelerationInWorld(const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {
        auto worldAngAccTmp = worldAngAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngAccTmp);
        auto chronoAngAcc = internal::Vector3dToChVector(worldAngAccTmp);
        m_chronoBody->SetWacc_par(chronoAngAcc); // FIXME : dans chrono, l'argument d'entree n'est pas const... -> fix Chrono
    }

    void FrBody_::SetAngularAccelerationInBody(const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {
        SetAngularAccelerationInWorld(ProjectVectorInWorld(bodyAngAcc, fc), fc);
    }

    AngularAcceleration FrBody_::GetAngularAccelerationInWorld(FRAME_CONVENTION fc) const {
        AngularAcceleration angAcc = internal::ChVectorToVector3d<AngularAcceleration>(m_chronoBody->GetWacc_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(angAcc);
        return angAcc;
    }

    AngularAcceleration FrBody_::GetAngularAccelerationInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody(GetAngularAccelerationInWorld(fc), fc);
    }

    Velocity FrBody_::GetVelocityInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetVelocityInWorldAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody_::GetVelocityInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        return ProjectVectorInWorld<Velocity>(GetVelocityInBodyAtPointInBody(bodyPoint, fc), fc);
    }

    Velocity FrBody_::GetVelocityInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        Position bodyPoint = GetPointPositionInBody(worldPoint);
        return GetVelocityInBodyAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody_::GetVelocityInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        Velocity bodyVel = GetVelocityInBody(fc);
        AngularVelocity bodyAngVel = GetAngularVelocityInBody(fc);
        Velocity pointVel = bodyVel + bodyAngVel.cross(bodyPoint);
    }

    Acceleration FrBody_::GetAccelerationInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        auto bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetAccelerationInWorldAtPointInBody(bodyPoint, fc);
    }

    Acceleration FrBody_::GetAccelerationInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        auto bodyPointTmp = bodyPoint;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(bodyPointTmp);

        Acceleration pointAcc = internal::ChVectorToVector3d<Acceleration>(
                m_chronoBody->PointAccelerationLocalToParent(internal::Vector3dToChVector(bodyPointTmp - GetCOG(NWU)))
                );

        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(pointAcc);
        return pointAcc;
    }

    Acceleration FrBody_::GetAccelerationInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        GetAccelerationInBodyAtPointInBody(GetPointPositionInBody(worldPoint, fc), fc);
    }

    Acceleration FrBody_::GetAccelerationInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        return ProjectVectorInBody(GetAccelerationInWorldAtPointInBody(bodyPoint, fc), fc);
    }

    
    void FrBody_::SetGeneralizedVelocityInWorldAtPointInWorld(const Position &worldPoint, const Velocity &worldVel,
                                                              const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        SetGeneralizedVelocityInWorldAtPointInBody(bodyPoint, worldVel, worldAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInWorldAtPointInBody(const Position &bodyPoint, const Velocity &worldVel,
                                                             const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        Velocity bodyVel = ProjectVectorInBody<Velocity>(worldVel, fc);
        AngularVelocity bodyAngVel = ProjectVectorInBody<AngularVelocity>(worldAngVel, fc);
        SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInBodyAtPointInWorld(const Position &worldPoint, const Velocity &bodyVel,
                                                             const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInBodyAtPointInBody(const Position &bodyPoint, const Velocity &bodyVel,
                                                            const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        SetAngularVelocityInBody(bodyAngVel, fc);
        Position PG = GetCOG(fc) - bodyPoint;
        Velocity cogVel = bodyVel + bodyAngVel.cross(PG);
        SetCOGVelocityInBody(cogVel, fc);
    }







    void
    FrBody_::SetGeneralizedAccelerationInWorldAtPointInWorld(const Position &worldPoint, const Acceleration &worldAcc,
                                                             const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {


        // Voir la methode TransformLocaToParent() de ChFrameMoving pour le transport d'acceleration de point

//        // pos_dtdt
//        parent.coord_dtdt.pos =
//                PointAccelerationLocalToParent(local.coord.pos, local.coord_dt.pos, local.coord_dtdt.pos);
//
//        // rot_dt
//        parent.coord_dt.rot = coord_dt.rot % local.coord.rot + this->coord.rot % local.coord_dt.rot;
//
//        // rot_dtdt
//        parent.coord_dtdt.rot = coord_dtdt.rot % local.coord.rot + (coord_dt.rot % local.coord_dt.rot) * 2 +
//                                this->coord.rot % local.coord_dtdt.rot;



//      PointAccelerationLocalToParent :

        // Calule l'acceleration d'un point fixe au corps odnt les coords sont donnees en local au corps. L'acceleration
        // est exprimee dans le repere world

//        return coord_dtdt.pos +
//               ((coord_dtdt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2) +
//               ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2);

        // Formule :
        // Acc_P_world = Acc_G_world + bodyQuat_pp *


    }

    void
    FrBody_::SetGeneralizedAccelerationInWorldAtPointInBody(const Position &bodyPoint, const Acceleration &worldAcc,
                                                            const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {

    }

    void
    FrBody_::SetGeneralizedAccelerationInBodyAtPointInWorld(const Position &worldPoint, const Acceleration &bodyAcc,
                                                            const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {

    }

    void FrBody_::SetGeneralizedAccelerationInBodyAtPointInBody(const Position &bodyPoint, const Acceleration &bodyAcc,
                                                                const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {

    }

    Velocity FrBody_::GetApparentVelocityInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) {
        return Velocity();
    }

    Velocity FrBody_::GetApparentVelocityInWorldAtPointInWorld(const Position &bodyPoint, FRAME_CONVENTION fc) {
        return Velocity();
    }

    Velocity FrBody_::GetApparentVelocityInBodyAtPointInBody(const Position &worldPoint, FRAME_CONVENTION fc) {
        return Velocity();
    }

    Velocity FrBody_::GetApparentVelocityInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) {
        return Velocity();
    }

    double FrBody_::GetApparentAngle(FLUID_TYPE ft, FRAME_CONVENTION fc) {
        return 0;
    }



    void FrBody_::_SetPointPosition(const Position& point, FRAME pointFrame, const Position& pos, FRAME posFrame, FRAME_CONVENTION fc) {

        Position WB = GetPosition(fc);

        Position WP;
        if (pointFrame == BODY) {
            // we have BP
            WP = GetPointPositionInWorld(point, fc);
        } else {
            WP = point;
        }

        Position WT;
        if (posFrame == BODY) {
            // we have BT
            WT = GetPointPositionInWorld(pos, fc);
        } else {
            WT = pos;
        }

        Position PT = WT - WP;

        TranslateInWorld(PT, fc);

    }


}  // end namespace frydom