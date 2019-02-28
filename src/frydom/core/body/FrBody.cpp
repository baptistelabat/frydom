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



#include "FrBody.h"

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"

#include "frydom/core/math/FrMatrix.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/asset/FrAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/geographicServices/FrGeographicServices.h"
#include "frydom/asset/FrForceAsset.h"


namespace frydom {

    namespace internal {

        FrBodyBase::FrBodyBase(FrBody *body) : chrono::ChBodyAuxRef(), m_frydomBody(body) {}

        void FrBodyBase::SetupInitial() {}

        void FrBodyBase::Update(bool update_assets) {
            chrono::ChBodyAuxRef::Update(update_assets);
            m_frydomBody->Update();
        }

        void FrBodyBase::UpdateAfterMove() {

            auto auxref_to_cog = (chrono::ChFrameMoving<double>)GetFrame_REF_to_COG();

            chrono::ChFrameMoving<double> auxref_to_abs;
            this->TransformLocalToParent(auxref_to_cog, auxref_to_abs);

            SetFrame_REF_to_abs(auxref_to_abs);

            chrono::ChBodyAuxRef::Update(true);

            // Updating markers
            UpdateMarkers(GetChTime());
        }

        void FrBodyBase::UpdateMarkerPositionToCOG(const chrono::ChVector<> newCOG) {

            chrono::ChVector<double> position;

            for (auto& marker : GetMarkerList()) {
                position = marker->GetPos() - newCOG;
                marker->Impose_Rel_Coord(chrono::ChCoordsys<double>(position));
            }
            UpdateMarkers(GetChTime());
        }

        void FrBodyBase::RemoveAsset(std::shared_ptr<chrono::ChAsset> asset) { //taken from RemoveForce
            // trying to remove objects not previously added?
            assert(std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(), asset) !=
                           assets.end());

            // warning! linear time search
            assets.erase(
                    std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(), asset));
        }

    }  // end namespace frydom::internal

    FrBody::FrBody() {

        SetLogged(true);

        m_chronoBody = std::make_shared<internal::FrBodyBase>(this);
        m_chronoBody->SetMaxSpeed(DEFAULT_MAX_SPEED);
        m_chronoBody->SetMaxWvel(DEFAULT_MAX_ROTATION_SPEED);

        m_DOFMask = std::make_unique<FrBodyDOFMask>();
    }

    FrOffshoreSystem* FrBody::GetSystem() const {
        return m_system;
    }

    void FrBody::SetName(const char *name) {
        m_chronoBody->SetName(name);
    }

    const char* FrBody::GetName() const {
        return m_chronoBody->GetName();
    }

    void FrBody::SetFixedInWorld(bool state) {
        m_chronoBody->SetBodyFixed(state);
    }

    void FrBody::SetupInitial() {
        m_chronoBody->SetupInitial();
        Initialize();
    }

    void FrBody::Initialize() {

        // Initializing forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->Initialize();
        }

        // Initializing nodes
        auto nodeIter = node_begin();
        for (; nodeIter != node_end(); nodeIter++) {
            (*nodeIter)->Initialize();
        }

        // BodyDOF constraints Initialization
        if (m_DOFMask->HasLockedDOF()) {
            InitializeLockedDOF();
        }

    }

    void FrBody::StepFinalize() {
        // StepFinalize of forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->StepFinalize();
        }

        // Send the message to the logging system
        if (IsLogged()) {
            m_message->Serialize();
            m_message->Send();
        }
    }

    void FrBody::Update() {
        // TODO

    }

    void FrBody::SetSmoothContact() {
        auto materialSurface = std::make_shared<chrono::ChMaterialSurfaceSMC>();
        m_chronoBody->SetMaterialSurface(materialSurface);
        m_contactType = CONTACT_TYPE::SMOOTH_CONTACT;
    }

    void FrBody::SetNonSmoothContact() {
        auto materialSurface = std::make_shared<chrono::ChMaterialSurfaceNSC>();
        m_chronoBody->SetMaterialSurface(materialSurface);
        m_contactType = CONTACT_TYPE::NONSMOOTH_CONTACT;
    }

    void FrBody::SetContactMethod(CONTACT_TYPE contactType) {
        switch (contactType) {
            case CONTACT_TYPE::SMOOTH_CONTACT:
                SetSmoothContact();
                break;
            case CONTACT_TYPE::NONSMOOTH_CONTACT:
                SetNonSmoothContact();
                break;
        }
    }

    FrBody::CONTACT_TYPE FrBody::GetContactType() const {
        return m_contactType;
    }


    // Force linear iterators
    FrBody::ForceIter FrBody::force_begin() {
        return m_externalForces.begin();
    }

    FrBody::ConstForceIter FrBody::force_begin() const {
        return m_externalForces.cbegin();
    }

    FrBody::ForceIter FrBody::force_end() {
        return m_externalForces.end();
    }

    FrBody::ConstForceIter FrBody::force_end() const {
        return m_externalForces.cend();
    }

    // Node linear iterators
    FrBody::NodeIter FrBody::node_begin() {
        return m_nodes.begin();
    }

    FrBody::ConstNodeIter FrBody::node_begin() const {
        return m_nodes.cbegin();
    }

    FrBody::NodeIter FrBody::node_end() {
        return m_nodes.end();
    }

    FrBody::ConstNodeIter FrBody::node_end() const {
        return m_nodes.cend();
    }

    void FrBody::AddMeshAsset(std::string obj_filename) {

        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        AddMeshAsset(mesh);
    }

    void FrBody::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        m_chronoBody->AddAsset(shape);
    }

    void FrBody::AddAsset(std::shared_ptr<FrAsset> asset) {
        m_assets.push_back(asset);
        m_chronoBody->AddAsset(asset->GetChronoAsset());
    }

    void FrBody::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrBody::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        m_chronoBody->AddAsset(colorAsset);
    }

    double FrBody::GetMass() const {
        return m_chronoBody->GetMass();
    }

    FrInertiaTensor FrBody::GetInertiaTensor(FRAME_CONVENTION fc) const {
        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        SplitMatrix33IntoCoeffs(internal::ChMatrix33ToMatrix33(m_chronoBody->GetInertia()),
                Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
        if (IsNED(fc)) {
            internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
        }

        return {GetMass(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame(GetCOG(fc), FrRotation(), fc), fc};
    }

    void FrBody::SetInertiaTensor(const FrInertiaTensor &inertia) {

        m_chronoBody->SetMass(inertia.GetMass());

        SetCOG(inertia.GetCOGPosition(NWU), NWU);

        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        inertia.GetInertiaCoeffs(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

        m_chronoBody->SetInertiaXX(chrono::ChVector<double>(Ixx, Iyy, Izz));
        m_chronoBody->SetInertiaXY(chrono::ChVector<double>(Ixy, Ixz, Iyz));

    }

    void FrBody::AllowCollision(bool isColliding) {
        m_chronoBody->SetCollide(isColliding);
    }

    void FrBody::AddBoxShape(double xSize, double ySize, double zSize) {
        auto shape = std::make_shared<chrono::ChBoxShape>();
        shape->GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
        m_chronoBody->AddAsset(shape);
    }

    void FrBody::AddCylinderShape(double radius, double height) {
        auto shape = std::make_shared<chrono::ChCylinderShape>();
        shape->GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
        shape->GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
        shape->GetCylinderGeometry().rad = radius;
        m_chronoBody->AddAsset(shape);
    }

    void FrBody::AddSphereShape(double radius) {
        auto shape = std::make_shared<chrono::ChSphereShape>();
        shape->GetSphereGeometry().rad = radius;
        m_chronoBody->AddAsset(shape);
    }

    void FrBody::ActivateSpeedLimits(bool activate) {
        m_chronoBody->SetLimitSpeed(activate);
    }

    void FrBody::SetMaxSpeed(double maxSpeed_ms) {
        m_chronoBody->SetMaxSpeed((float)maxSpeed_ms);
        ActivateSpeedLimits(true);
    }

    void FrBody::SetMaxRotationSpeed(double wMax_rads) {
        m_chronoBody->SetMaxWvel((float)wMax_rads);
        ActivateSpeedLimits(true);
    }

    void FrBody::RemoveGravity(bool val) { // TODO : ajouter la force d'accumulation a l'initialisation --> cas ou le systeme n'a pas encore ete precise pour la gravite...
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

    void FrBody::AddExternalForce(std::shared_ptr<frydom::FrForce> force) {
        /// This subroutine is used for adding the hydrodynamic loads.
        m_chronoBody->AddForce(force->GetChronoForce());  // FrBody is a friend class of FrForce

        force->m_body = this;
        m_externalForces.push_back(force);

    }

    void FrBody::RemoveExternalForce(std::shared_ptr<FrForce> force) {
        m_chronoBody->RemoveForce(force->GetChronoForce());

        m_externalForces.erase(
                std::find<std::vector<std::shared_ptr<FrForce>>::iterator>(m_externalForces.begin(), m_externalForces.end(), force));

        if (force->m_forceAsset!=nullptr) {
            m_chronoBody->RemoveAsset(force->m_forceAsset->GetChronoAsset());

            bool asserted=false;
            for (int ia=0;ia<m_assets.size();++ia){
                if (m_assets[ia]==force->m_forceAsset){
                    m_assets.erase(m_assets.begin()+ia);
                    asserted=true;
                }
            }
            assert(asserted);
            force->m_forceAsset=nullptr;

        }


        force->m_body = nullptr;
    }

    void FrBody::RemoveAllForces() {
        m_chronoBody->RemoveAllForces();
        for (auto forceIter=force_begin(); forceIter!=force_end(); forceIter++) {
            (*forceIter)->m_body = nullptr;
        }
        m_externalForces.clear();
    }

    // ##CC adding for monitoring load

    Force FrBody::GetTotalForceInWorld(FRAME_CONVENTION fc) const {
        auto chronoForce = m_chronoBody->Get_Xforce();
        return Force(chronoForce.x(), chronoForce.y(), chronoForce.z());
    }

    Torque FrBody::GetTotalTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
        auto chronoTorque = m_chronoBody->Get_Xtorque();
        return Torque(chronoTorque.x(), chronoTorque.y(), chronoTorque.z());
    }


    // Nodes

    std::shared_ptr<FrNode> FrBody::NewNode() {
        return std::make_shared<FrNode>(this);
    }

    void FrBody::SetCOG(const Position& bodyPos, FRAME_CONVENTION fc) {
        FrFrame cogFrame;
        cogFrame.SetPosition(bodyPos, fc);
        m_chronoBody->UpdateMarkerPositionToCOG(internal::Vector3dToChVector(cogFrame.GetPosition(NWU)));
        m_chronoBody->SetFrame_COG_to_REF(internal::FrFrame2ChFrame(cogFrame));
    }

    Position FrBody::GetCOG(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_COG_to_REF().GetPos()); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    Position FrBody::GetPosition(FRAME_CONVENTION fc) const {
        Position refPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_REF_to_abs().GetPos());
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(refPos);
        return refPos;
    }

    FrGeographicCoord FrBody::GetGeoPosition(FRAME_CONVENTION fc) const {
        return CartToGeo(GetPosition(fc),fc);
    }

    void FrBody::SetPosition(const Position &worldPos, FRAME_CONVENTION fc) {

        /// This subroutine sets the initial position of a body in world.

        auto bodyFrame = GetFrame();
        bodyFrame.SetPosition(worldPos, fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(bodyFrame));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::SetGeoPosition(const FrGeographicCoord& geoCoord) {
        SetPosition(GeoToCart(geoCoord,NWU),NWU);
    }

    FrRotation FrBody::GetRotation() const {
        return FrRotation(GetQuaternion());
    }

    void FrBody::SetRotation(const FrRotation &rotation) {
        SetRotation(rotation.GetQuaternion());
    }

    FrUnitQuaternion FrBody::GetQuaternion() const {
        return internal::Ch2FrQuaternion(m_chronoBody->GetRot());
    }

    void FrBody::SetRotation(const FrUnitQuaternion &quaternion) {
        Position bodyWorldPos = GetPosition(NWU);
        m_chronoBody->SetRot(internal::Fr2ChQuaternion(quaternion));
        SetPosition(bodyWorldPos, NWU);
    }

    FrFrame FrBody::GetFrame() const {
        FrFrame bodyRefFrame;
        bodyRefFrame.SetPosition(GetPosition(NWU), NWU);
        bodyRefFrame.SetRotation(GetQuaternion());
        return bodyRefFrame;
    }

    void FrBody::SetFrame(const FrFrame &worldFrame) {
        SetPosition(worldFrame.GetPosition(NWU), NWU);
        SetRotation(worldFrame.GetQuaternion());
    }

    FrFrame FrBody::GetFrameAtPoint(const Position& bodyPoint, FRAME_CONVENTION fc) {
        FrFrame pointFrame;
        pointFrame.SetPosition(GetPointPositionInWorld(bodyPoint, fc), fc);
        pointFrame.SetRotation(GetQuaternion());
        return pointFrame;
    }

    FrFrame FrBody::GetFrameAtCOG(FRAME_CONVENTION fc) {
        return GetFrameAtPoint(GetCOG(fc), fc);
    }

    Position FrBody::GetPointPositionInWorld(const Position &bodyPos, FRAME_CONVENTION fc) const {
        return GetPosition(fc) + ProjectVectorInWorld<Position>(bodyPos, fc);
    }

    Position FrBody::GetPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Position>(worldPos - GetPosition(fc), fc);
    }

    Position FrBody::GetCOGPositionInWorld(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetPos());
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }


    FrGeographicCoord FrBody::GetGeoPointPositionInWorld(const Position& bodyPos, FRAME_CONVENTION fc) const {
        return CartToGeo(GetPointPositionInWorld(bodyPos, fc), fc);
    }

    FrGeographicCoord FrBody::GetGeoPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
        return CartToGeo(GetPointPositionInBody(worldPos, fc), fc);
    }

    FrGeographicCoord FrBody::GetCOGGeoPosition() const {
        return CartToGeo(GetCOGPositionInWorld(NWU), NWU);
    }


    void FrBody::SetPositionOfBodyPoint(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc) {
        Position bodyWorldPos = GetPosition(fc);
        Position worldPointPos = GetPointPositionInWorld(bodyPoint, fc);

        Translation translation = worldPos - worldPointPos;
        TranslateInWorld(translation, fc);
    }

    void FrBody::TranslateInWorld(const Translation &worldTranslation, FRAME_CONVENTION fc) {
        auto refFrame = GetFrame();
        refFrame.SetPosition(refFrame.GetPosition(fc) + worldTranslation, fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(refFrame));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInWorld(Translation(x, y, z), fc);
    }

    void FrBody::TranslateInBody(const Translation &bodyTranslation, FRAME_CONVENTION fc) {
        auto refFrame = GetFrame();
        refFrame.SetPosition(refFrame.GetPosition(fc) + ProjectVectorInWorld<Position>(bodyTranslation, fc), fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(refFrame));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInBody(Translation(x, y, z), fc);
    }

    void FrBody::Rotate(const FrRotation &relRotation) {
        SetRotation(GetRotation() * relRotation);
    }

    void FrBody::Rotate(const FrUnitQuaternion &relQuaternion) {
        SetRotation(GetQuaternion() * relQuaternion);
    }

    void FrBody::RotateAroundPointInWorld(const FrRotation& rot, const Position& worldPos, FRAME_CONVENTION fc) {
        RotateAroundPointInWorld(rot.GetQuaternion(), worldPos, fc);
    }

    void FrBody::RotateAroundPointInBody(const FrRotation& rot, const Position& bodyPos, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot.GetQuaternion(), bodyPos, fc);
    }

    void FrBody::RotateAroundPointInWorld(const FrUnitQuaternion& rot, const Position& worldPos, FRAME_CONVENTION fc) {
        Position bodyPos = GetPointPositionInBody(worldPos, fc);
        Rotate(rot);
        SetPositionOfBodyPoint(bodyPos, worldPos, fc);
    }

    void FrBody::RotateAroundPointInBody(const FrUnitQuaternion& rot, const Position& bodyPos, FRAME_CONVENTION fc) {
        Position worldPos = GetPointPositionInWorld(bodyPos, fc);
        Rotate(rot);
        SetPositionOfBodyPoint(bodyPos, worldPos, fc);
    }

    void FrBody::RotateAroundCOG(const FrRotation& rot, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot, GetCOG(fc), fc);
    }

    void FrBody::RotateAroundCOG(const FrUnitQuaternion& rot, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot, GetCOG(fc), fc);
    }

    void FrBody::SetGeneralizedVelocityInWorld(const Velocity& worldVel, const AngularVelocity& worldAngVel,
            FRAME_CONVENTION fc) {
        SetGeneralizedVelocityInWorldAtPointInBody(Position(0., 0., 0.), worldVel, worldAngVel, fc);
    }

    void FrBody::SetGeneralizedVelocityInBody(const Velocity& bodyVel, const AngularVelocity& bodyAngVel,
            FRAME_CONVENTION fc) {
        SetGeneralizedVelocityInBodyAtPointInBody(Position(0., 0., 0.), bodyVel, bodyAngVel, fc);
    }

    Velocity FrBody::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity bodyVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(bodyVel);
        return bodyVel;
    }

    Velocity FrBody::GetVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetVelocityInWorld(fc), fc);
    }

    void FrBody::SetVelocityInWorldNoRotation(const Velocity &worldVel, FRAME_CONVENTION fc) {
        auto worldVelTmp = worldVel;
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(worldVelTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldVelTmp);
        coord.rot.SetNull();
        m_chronoBody->SetCoord_dt(coord);
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::SetVelocityInBodyNoRotation(const Velocity &bodyVel, FRAME_CONVENTION fc) {
        SetVelocityInWorldNoRotation(ProjectVectorInWorld(bodyVel, fc), fc);
    }

    Velocity FrBody::GetCOGVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity cogVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetCoord_dt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(cogVel);
        return cogVel;
    }

    Velocity FrBody::GetCOGVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetCOGVelocityInWorld(fc), fc);
    }

    void FrBody::SetAccelerationInWorldNoRotation(const Acceleration &worldAcc, FRAME_CONVENTION fc) {
        auto worldAccTmp = worldAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(worldAccTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldAccTmp);
        coord.rot.SetNull();
        m_chronoBody->SetCoord_dtdt(coord);
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::SetAccelerationInBodyNoRotation(const Acceleration &bodyAcc, FRAME_CONVENTION fc) {
        SetAccelerationInWorldNoRotation(ProjectVectorInWorld<Acceleration>(bodyAcc, fc), fc);
    }

    Acceleration FrBody::GetCOGAccelerationInWorld(FRAME_CONVENTION fc) const {
        Acceleration cogAcc = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->GetCoord_dtdt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(cogAcc);
        return cogAcc;
    }

    Acceleration FrBody::GetCOGAccelerationInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Acceleration>(GetCOGAccelerationInWorld(fc), fc);
    }

    void FrBody::SetAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        auto worldAngVelTmp = worldAngVel;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngVelTmp);
        m_chronoBody->SetWvel_par(internal::Vector3dToChVector(worldAngVelTmp));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::SetCOGAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        auto worldAngVelTmp = worldAngVel;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngVelTmp);
        m_chronoBody->SetWvel_par(internal::Vector3dToChVector(worldAngVelTmp));
    }

    void FrBody::SetAngularVelocityInBody(const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        SetAngularVelocityInWorld(ProjectVectorInWorld(bodyAngVel, fc), fc);
    }

    AngularVelocity FrBody::GetAngularVelocityInWorld(FRAME_CONVENTION fc) const {
        AngularVelocity angVel = internal::ChVectorToVector3d<AngularVelocity>(m_chronoBody->GetWvel_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(angVel);
        return angVel;
    }

    AngularVelocity FrBody::GetAngularVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<AngularVelocity>(GetAngularVelocityInWorld(fc), fc);
    }

    void FrBody::SetAngularAccelerationInWorld(const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {
        auto worldAngAccTmp = worldAngAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(worldAngAccTmp);
        auto chronoAngAcc = internal::Vector3dToChVector(worldAngAccTmp);
        m_chronoBody->SetWacc_par(chronoAngAcc); // FIXME : dans chrono, l'argument d'entree n'est pas const... -> fix Chrono
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody::SetAngularAccelerationInBody(const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {
        SetAngularAccelerationInWorld(ProjectVectorInWorld(bodyAngAcc, fc), fc);
    }

    AngularAcceleration FrBody::GetAngularAccelerationInWorld(FRAME_CONVENTION fc) const {
        AngularAcceleration angAcc = internal::ChVectorToVector3d<AngularAcceleration>(m_chronoBody->GetWacc_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(angAcc);
        return angAcc;
    }

    AngularAcceleration FrBody::GetAngularAccelerationInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody(GetAngularAccelerationInWorld(fc), fc);
    }

    Velocity FrBody::GetVelocityInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetVelocityInWorldAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody::GetVelocityInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        return ProjectVectorInWorld<Velocity>(GetVelocityInBodyAtPointInBody(bodyPoint, fc), fc);
    }

    Velocity FrBody::GetVelocityInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetVelocityInBodyAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody::GetVelocityInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        Velocity bodyVel = GetVelocityInBody(fc);
        AngularVelocity bodyAngVel = GetAngularVelocityInBody(fc);
        return bodyVel + bodyAngVel.cross(bodyPoint);
    }

    Acceleration FrBody::GetAccelerationInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        auto bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetAccelerationInWorldAtPointInBody(bodyPoint, fc);
    }

    Acceleration FrBody::GetAccelerationInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        auto bodyPointTmp = bodyPoint;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(bodyPointTmp);

        Acceleration pointAcc = internal::ChVectorToVector3d<Acceleration>(
                m_chronoBody->PointAccelerationLocalToParent(internal::Vector3dToChVector(bodyPointTmp - GetCOG(NWU)))
                );

        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(pointAcc);
        return pointAcc;
    }

    Acceleration FrBody::GetAccelerationInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        return GetAccelerationInBodyAtPointInBody(GetPointPositionInBody(worldPoint, fc), fc);
    }

    Acceleration FrBody::GetAccelerationInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        return ProjectVectorInBody(GetAccelerationInWorldAtPointInBody(bodyPoint, fc), fc);
    }


    void FrBody::SetGeneralizedVelocityInWorldAtPointInWorld(const Position &worldPoint, const Velocity &worldVel,
                                                              const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        SetGeneralizedVelocityInWorldAtPointInBody(bodyPoint, worldVel, worldAngVel, fc);
    }

    void FrBody::SetGeneralizedVelocityInWorldAtPointInBody(const Position &bodyPoint, const Velocity &worldVel,
                                                             const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        Velocity bodyVel = ProjectVectorInBody<Velocity>(worldVel, fc);
        AngularVelocity bodyAngVel = ProjectVectorInBody<AngularVelocity>(worldAngVel, fc);
        SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
    }

    void FrBody::SetGeneralizedVelocityInBodyAtPointInWorld(const Position &worldPoint, const Velocity &bodyVel,
                                                             const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
    }

    void FrBody::SetGeneralizedVelocityInBodyAtPointInBody(const Position &bodyPoint, const Velocity &bodyVel,
                                                            const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {

        Position PG = GetCOG(fc) - bodyPoint;
        Velocity cogVel = bodyVel + bodyAngVel.cross(PG);
        SetVelocityInBodyNoRotation(cogVel, fc);
        SetAngularVelocityInBody(bodyAngVel, fc);
    }

    void FrBody::SetGeneralizedAccelerationInBodyAtCOG(const Acceleration &bodyAcc, const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {
        SetAccelerationInBodyNoRotation(bodyAcc, fc);
        SetAngularAccelerationInBody(bodyAngAcc, fc);
    }

    void FrBody::SetGeneralizedAccelerationInWorldAtCOG(const Acceleration &worldAcc, const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {
        SetAccelerationInWorldNoRotation(worldAcc, fc);
        SetAngularAccelerationInWorld(worldAngAcc, fc);
    }

    void FrBody::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
        geoCoord = CartToGeo(cartPos, fc);
    }
    void FrBody::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) {
        geoCoord = CartToGeo(cartPos, fc);
    }

    FrGeographicCoord FrBody::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) const {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->CartToGeo(cartPos, fc);
    }

    FrGeographicCoord FrBody::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->CartToGeo(cartPos, fc);
    }


    void FrBody::GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc) const {
        cartPos = GeoToCart(geoCoord, fc);
    }

    void FrBody::GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc) {
        cartPos = GeoToCart(geoCoord, fc);
    }

    Position FrBody::GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) const {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->GeoToCart(geoCoord, fc);
    }

    Position FrBody::GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->GeoToCart(geoCoord, fc);
    }

    void FrBody::InitializeLockedDOF() {

        // TODO : voir si n'accepte pas de definir des offset sur les ddl bloques...

        // Getting the markers that enter in the link

        // Body marker placed at the current COG body position  // TODO : voir si on se donne d'autres regles que le COG...
        auto bodyNode = NewNode();

        auto cogPositionInWorld= GetCOGPositionInWorld(NWU);
        auto bodyOrientationInWorld = GetQuaternion();

        auto bodyNodeFrameInWorld = FrFrame(cogPositionInWorld, bodyOrientationInWorld, NWU);
        bodyNode->SetFrameInWorld(bodyNodeFrameInWorld);

        // World Marker placed at the current COG body position
        auto worldNode = GetSystem()->GetWorldBody()->NewNode();
        worldNode->SetFrameInBody(bodyNodeFrameInWorld);

        // Creating the link
        m_DOFLink = std::make_shared<FrLink>(worldNode, bodyNode, GetSystem());

        // Initializing the link with the DOFMask
        m_DOFLink->InitializeWithBodyDOFMask(m_DOFMask.get());

        // Adding the link to the system
        m_system->AddLink(m_DOFLink);

    }

    FrBodyDOFMask* FrBody::GetDOFMask() {
        return m_DOFMask.get();
    }

    void FrBody::InitializeLog() {

        auto logPath = m_system->GetPathManager()->BuildBodyPath(this);

        // Add the fields
        m_message->AddField<double>("time", "s", "Current time of the simulation", [this] () { return m_chronoBody->GetChTime();});

        // Initialize the message
        FrObject::InitializeLog(logPath);

        // Initializing forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->InitializeLog();
        }

        // Initializing nodes
        auto nodeIter = node_begin();
        for (; nodeIter != node_end(); nodeIter++) {
            (*nodeIter)->InitializeLog();
        }


    }


}  // end namespace frydom
