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

    void FrBody_::SetMass(double mass) {
        m_chronoBody->SetMass(mass);
    }

    double FrBody_::GetMass() const {
        return m_chronoBody->GetMass();
    }

    void FrBody_::SetMasInTons(double mass) {
        m_chronoBody->SetMass(mass*1e3);
    }

    void FrBody_::SetDiagonalInertiasWRT_COG(double Ixx, double Iyy, double Izz) {
        m_chronoBody->SetInertiaXX(chrono::ChVector<double>(Ixx, Iyy, Izz));
    }

    void FrBody_::SetOffDiagonalInertiasWRT_COG(double Ixy, double Ixz, double Iyz) {
        m_chronoBody->SetInertiaXY(chrono::ChVector<double>(Ixy, Ixz, Iyz));
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
        m_chronoBody->SetMaxSpeed(maxSpeed_ms);
    }

    void FrBody_::SetMaxRotationSpeed(double wMax_rads) {
        m_chronoBody->SetMaxWvel(wMax_rads);
    }

    void FrBody_::RemoveGravity(bool val) {  // FIXME : cet ajout doit l'etre lors de l'initialisation !!! --> booleen en attribut
        if (val) {
            m_chronoBody->Accumulate_force(GetMass() * chrono::ChVector<double>(0., 0., 9.81),
                    chrono::ChVector<double>(), true);
            // TODO : aller chercher la gravite dans systeme !!!
        }
    }




    //        // TODO : Analyse de ce qui est fait pour ChBodyAuxRef :::
//
//        /// Set the COG frame with respect to the auxiliary reference frame.
//        /// Note that this also moves the body absolute COG (the REF is fixed).
//        /// The position of contained ChMarker objects, if any, is not changed with respect
//        /// to the reference.
//        m_chronoBody->SetFrame_COG_to_REF()
//
//        /// Set the auxiliary reference frame with respect to the COG frame.
//        /// Note that this does not move the body absolute COG (the COG is fixed).
//        m_chronoBody->SetFrame_REF_to_COG()
//
//        /// Set the auxiliary reference frame with respect to the absolute frame.
//        /// This moves the entire body; the body COG is rigidly moved as well.
//        m_chronoBody->SetFrame_REF_to_abs()
//
//        // Impose the translation (the COG position)
//        m_chronoBody->SetPos()
//
//        // Set the linear speed
//        m_chronoBody->SetPos_dt()
//
//        // Set the linear acceleration
//        m_chronoBody->SetPos_dtdt()
//
//        // Impose the rotation
//        m_chronoBody->SetRot()
//
//        // Set the rotation speed (quaternion)
//        m_chronoBody->SetRot_dt()
//
//        // Set the rotation speed (angular speed in local coordinate system)
//        m_chronoBody->SetWvel_loc()
//
//        // Set the rotation speed (angular speed in parent coordinate system)
//        m_chronoBody->SetWvel_par()
//
//        m_chronoBody->SetRot_dtdt() // quaternion
//        m_chronoBody->SetWacc_loc()
//        m_chronoBody->SetWacc_par(
//
//        // TODO : les fonctions precedentes ont leur contre partie GetXXX()
//
//
//
//        // Fonctions pour limiter les vitesses
//        m_chronoBody->SetMaxSpeed()
//        m_chronoBody->SetMaxWvel()
//        m_chronoBody->SetLimitSpeed()
//
//
//        /// Get the rigid body coordinate system that represents
//        /// the GOG (Center of Gravity). The mass and inertia tensor
//        /// are defined respect to this coordinate system, that is also
//        /// assumed the default main coordinates of the body.
//        /// By default, doing mybody.GetPos() etc. is like mybody.GetFrame_COG_abs().GetPos() etc.
//        m_chronoBody->GetFrame_COG_to_abs()
//
//        /// Get the COG frame with respect to the auxiliary reference frame.
//        m_chronoBody->GetFrame_COG_to_REF()
//
//        /// Get the auxiliary reference frame with respect to the absolute frame.
//        /// Note that, in general, this is different from GetFrame_COG_to_abs().
//        m_chronoBody->GetFrame_REF_to_abs()
//
//        /// Get the auxiliary reference frame with respect to the COG frame.
//        m_chronoBody->GetFrame_REF_to_COG()



    void FrBody_::SetCOGLocalPosition(double x, double y, double z, FRAME_CONVENTION fc) {  // OK

        auto cogFrame = chrono::ChFrame<double>();
        cogFrame.SetPos(internal::MakeNWUChVector(x, y, z, fc));
        m_chronoBody->SetFrame_COG_to_REF(cogFrame);

        // FIXME : transporter la matrice d'inertie en meme temps !!!!

        auto inertiaMat = m_chronoBody->GetInertia();






        m_chronoBody->Update();  // To make auxref_to_abs up to date
    }

    void FrBody_::SetCOGLocalPosition(Position position, FRAME_CONVENTION fc) {  // OK
        SetCOGLocalPosition(position[0], position[1], position[2], fc);
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

    Position FrBody_::GetCOGLocalPosition(FRAME_CONVENTION fc) const {  // OK
        auto cogPos = m_chronoBody->GetFrame_COG_to_REF().GetPos(); // In NWU

        auto frPos = internal::ChVectorToVector3d<Position>(cogPos);

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

    FrRotation_ FrBody_::GetAbsRotation(FRAME_CONVENTION fc) const {
        return FrRotation_(GetAbsQuaternion(fc));
    }

    FrQuaternion_ FrBody_::GetAbsQuaternion(FRAME_CONVENTION fc) const {
        return internal::Ch2FrQuaternion(m_chronoBody->GetFrame_REF_to_abs().GetRot());
    }

    void FrBody_::GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {
        GetAbsRotation(fc).GetEulerAngles_RADIANS(phi, theta, psi, seq, fc);
    }

    void FrBody_::GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {
        GetAbsRotation(fc).GetEulerAngles_DEGREES(phi, theta, psi, seq, fc);
    }

    void FrBody_::GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
        GetAbsRotation(fc).GetCardanAngles_RADIANS(phi, theta, psi, fc);
    }

    void FrBody_::GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
        GetAbsRotation(fc).GetCardanAngles_DEGREES(phi, theta, psi, fc);
    }

    void FrBody_::GetRotationAxisAngle(Direction &axis, double angle, FRAME_CONVENTION fc) const {
        GetAbsRotation(fc).GetAxisAngle(axis, angle, fc);
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

        auto cogVelocity = localReferenceVelocity + GL.Cross(omega);


        m_chronoBody->SetPos_dt(cogVelocity);

//        m_chronoBody->Get_gyro()






//        m_chronoBody->GetFrame_REF_to_abs().SetPos_dt(ChVector)


    }

    void FrBody_::SetAbsVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        // TODO
    }

    Velocity FrBody_::GetAbsVelocity(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::GetAbsVelocity(Velocity &velocity, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::GetAbsVelocity(double &vx, double &vy, double &vz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::SetLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::SetLocalVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        // TODO
    }

    Velocity FrBody_::GetLocalVelocity(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::GetLocalVelocity(Velocity &velocity, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::GetLocalVelocity(double &u, double &v, double &w, FRAME_CONVENTION fc) const {
        // TODO
    }

    Velocity FrBody_::GetAbsVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        // TODO
    }

    Velocity FrBody_::GetLocalVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const {
        // TODO
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
        auto cogAbsVel = GetCOGAbsVelocity(fc);

        // Projecting this velocity into relative coordinates
        auto absQuat = GetAbsQuaternion(fc);

        auto cogRelVel = absQuat.Rotate(cogAbsVel, fc);




        // FIXME : finir avec quelque chose de performant !!!!



//        m_chronoBody->GetWvel_par()








    }

    void FrBody_::SetCOGLocalVelocity(const Velocity &velocity, FRAME_CONVENTION fc) {
        // TODO
    }

    Velocity FrBody_::GetCOGLocalVelocity(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::GetCOGLocalVelocity(Velocity &velocity, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::GetCOGLocalVelocity(double &u, double &v, double &w, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::SetAbsRotationalVelocity(double wx, double wy, double wz, FRAME_CONVENTION fc) {  // OK si le COG est sur le frame
        if (IsNED(fc)) internal::SwapCoordinateConvention(wx, wy, wz); // Convert into NWU
        m_chronoBody->SetWvel_par(chrono::ChVector<double>(wx, wy, wz));
    }

    void FrBody_::SetAbsRotationalVelocity(const RotationalVelocity &omega, FRAME_CONVENTION fc) {
        // TODO
    }

    RotationalVelocity FrBody_::GetAbsRotationalVelocity(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::GetAbsRotationalVelocity(RotationalVelocity &omega, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrBody_::GetAbsRotationalVelocity(double &wx, double &wy, double &wz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrBody_::SetAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc) {
        // TODO
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