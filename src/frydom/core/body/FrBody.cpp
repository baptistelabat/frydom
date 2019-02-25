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
#include "frydom/core/common/FrNode.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/core/math/FrMatrix.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "frydom/core/common/FrFrame.h"

#include "frydom/core/force/FrForce.h"

#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"

#include <chrono/physics/ChLinkMotorLinearSpeed.h>  // FIXME : a retirer
#include "frydom/asset/FrAsset.h"
#include "frydom/asset/FrForceAsset.h"

#include "frydom/core/math/functions/FrFunctionBase.h"
#include "frydom/core/common/FrException.h"
//#include "FrCore.h"

#include "frydom/core/link/links_lib/FrLink.h"


namespace frydom {

//    void FrBody::AddNode(std::shared_ptr<FrNode> node) {
//        // Adding the node as a marker to the body
//        AddMarker(node);
//        // TODO: PUT Force related stuff here
//
//    }
//
//    std::shared_ptr<FrNode> FrBody::CreateNode() {
//
//        // Creating the node and updating its position
//        auto node = std::make_shared<FrNode>();
//        node->SetBody(this);  // FIXME: a priori, c'est deja fait dans AddMarker lors de l'appele a AddNode... A retirer
//        node->UpdateState();  // TODO: voir s'il est besoin d'appeler l'update...
//
//        AddNode(node);
//        return node;
//    }
//
//    std::shared_ptr<FrNode> FrBody::CreateNode(const chrono::ChVector<double> relpos) {
//
//        auto node = CreateNode();
//
////        node->SetPos(relpos);  // TODO: Voir a utiliser ImposeRelPos tel que demande sur la liste chrono
//        chrono::ChCoordsys<> coord;
//        coord.pos = relpos;
//        node->Impose_Rel_Coord(coord);
//        node-> UpdateState();
//
//        return node;
//    }
//
//
//    void FrBody::SetVisuMesh(std::shared_ptr<FrTriangleMeshConnected> mesh) {
//
//        m_visu_mesh = mesh;
//
//        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
//        shape->SetMesh(*mesh);
//        AddAsset(shape);
//
//    }
//
//    void FrBody::SetVisuMesh(std::string obj_filename) {
//        auto mesh=std::make_shared<FrTriangleMeshConnected>();
//        mesh->LoadWavefrontMesh(obj_filename);
//        SetVisuMesh(mesh);
//    }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    // REFACTORING ------------->>>>>>>>>>>>>>>


    #define DEFAULT_MAX_SPEED (float)10.
    #define DEFAULT_MAX_ROTATION_SPEED (float)(180.*DEG2RAD)

    namespace internal {

        _FrBodyBase::_FrBodyBase(FrBody_ *body) : chrono::ChBodyAuxRef(), m_frydomBody(body) {}

        void _FrBodyBase::SetupInitial() {
//            chrono::ChBodyAuxRef::SetupInitial();
        }

        void _FrBodyBase::Update(bool update_assets) {
            chrono::ChBodyAuxRef::Update(update_assets);
            m_frydomBody->Update();
        }

        void _FrBodyBase::UpdateAfterMove() {

            auto auxref_to_cog = (chrono::ChFrameMoving<double>)GetFrame_REF_to_COG();

            chrono::ChFrameMoving<double> auxref_to_abs;
            this->TransformLocalToParent(auxref_to_cog, auxref_to_abs);

            SetFrame_REF_to_abs(auxref_to_abs);

            chrono::ChBodyAuxRef::Update(true);

            // Updating markers
            UpdateMarkers(GetChTime());
        }

        void _FrBodyBase::UpdateMarkerPositionToCOG(const chrono::ChVector<> newCOG) {

            chrono::ChVector<double> position;

            for (auto& marker : GetMarkerList()) {
                position = marker->GetPos() - newCOG;
                marker->Impose_Rel_Coord(chrono::ChCoordsys<double>(position));
            }
            UpdateMarkers(GetChTime());
        }

        void _FrBodyBase::RemoveAsset(std::shared_ptr<chrono::ChAsset> asset) { //taken from RemoveForce
            // trying to remove objects not previously added?
            assert(std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(), asset) !=
                           assets.end());

            // warning! linear time search
            assets.erase(
                    std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(), asset));
        }

    }  // end namespace internal

    FrBody_::FrBody_() {
        m_chronoBody = std::make_shared<internal::_FrBodyBase>(this);
        m_chronoBody->SetMaxSpeed(DEFAULT_MAX_SPEED);
        m_chronoBody->SetMaxWvel(DEFAULT_MAX_ROTATION_SPEED);

        m_DOFMask = std::make_unique<FrBodyDOFMask>();
    }

    FrOffshoreSystem_* FrBody_::GetSystem() const {
        return m_system;
    }

    void FrBody_::SetName(const char *name) {
        m_chronoBody->SetName(name);
    }

    void FrBody_::SetFixedInWorld(bool state) {
        m_chronoBody->SetBodyFixed(state);
    }

    void FrBody_::SetupInitial() {
        m_chronoBody->SetupInitial();
        Initialize();
    }

    void FrBody_::Initialize() {

        // Initializing forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->Initialize();
        }

        // BodyDOF constraints Initialization
        if (m_DOFMask->HasLockedDOF()) {
            InitializeLockedDOF();
        }


        // TODO : initialiser les logs




    }

    void FrBody_::StepFinalize() {
        // TODO
        // StepFinalize of forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->StepFinalize();
        }



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

    void FrBody_::AddAsset(std::shared_ptr<FrAsset> asset) {
        m_assets.push_back(asset);
        m_chronoBody->AddAsset(asset->GetChronoAsset());
    }

    void FrBody_::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrBody_::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        m_chronoBody->AddAsset(colorAsset);
    }

    double FrBody_::GetMass() const {
        return m_chronoBody->GetMass();
    }

    FrInertiaTensor_ FrBody_::GetInertiaTensor(FRAME_CONVENTION fc) const {
        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        SplitMatrix33IntoCoeffs(internal::ChMatrix33ToMatrix33(m_chronoBody->GetInertia()),
                Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
        if (IsNED(fc)) {
            internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
        }

        return {GetMass(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame_(GetCOG(fc), FrRotation_(), fc), fc};
    }

    void FrBody_::SetInertiaTensor(const FrInertiaTensor_ &inertia) {

        m_chronoBody->SetMass(inertia.GetMass());

        SetCOG(inertia.GetCOGPosition(NWU), NWU);

        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        inertia.GetInertiaCoeffs(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

        m_chronoBody->SetInertiaXX(chrono::ChVector<double>(Ixx, Iyy, Izz));
        m_chronoBody->SetInertiaXY(chrono::ChVector<double>(Ixy, Ixz, Iyz));

    }

//    void FrBody_::SetInertiaParams(double mass,
//                          double Ixx, double Iyy, double Izz,
//                          double Ixy, double Ixz, double Iyz,
//                          const FrFrame_& coeffsFrame,
//                          const Position& cogPosition,
//                          FRAME_CONVENTION fc) {
//        SetInertiaParams(FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, coeffsFrame, cogPosition, fc));
//    }
//
//    void FrBody_::SetInertiaParams(double mass,
//                          double Ixx, double Iyy, double Izz,
//                          double Ixy, double Ixz, double Iyz,
//                          const FrFrame_& cogFrame,
//                          FRAME_CONVENTION fc) {
//        SetInertiaParams(FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, cogFrame, fc));
//    }

    void FrBody_::AllowCollision(bool isColliding) {
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
        ActivateSpeedLimits(true);
    }

    void FrBody_::SetMaxRotationSpeed(double wMax_rads) {
        m_chronoBody->SetMaxWvel((float)wMax_rads);
        ActivateSpeedLimits(true);
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
        /// This subroutine is used for adding the hydrodynamic loads.
        m_chronoBody->AddForce(force->GetChronoForce());  // FrBody_ is a friend class of FrForce_

        force->m_body = this;
        m_externalForces.push_back(force);

    }

    void FrBody_::RemoveExternalForce(std::shared_ptr<FrForce_> force) {
        m_chronoBody->RemoveForce(force->GetChronoForce());

        m_externalForces.erase(
                std::find<std::vector<std::shared_ptr<FrForce_>>::iterator>(m_externalForces.begin(), m_externalForces.end(), force));

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

//            { // just to make sure this shared pointer is not used anywhere else.
//                auto sharedAsset = std::make_shared<FrForceAsset_>(force->m_forceAsset);
//                m_assets.erase(
//                        std::find<std::vector<std::shared_ptr<FrAsset>>::iterator>(m_assets.begin(), m_assets.end(),
//                                                                                   sharedAsset));
//            }
        }


        force->m_body = nullptr;
    }

    void FrBody_::RemoveAllForces() {
        m_chronoBody->RemoveAllForces();
        for (auto forceIter=force_begin(); forceIter!=force_end(); forceIter++) {
            (*forceIter)->m_body = nullptr;
        }
        m_externalForces.clear();
    }

    // ##CC adding for monitoring load

    Force FrBody_::GetTotalForceInWorld(FRAME_CONVENTION fc) const {
        auto chronoForce = m_chronoBody->Get_Xforce();
        return Force(chronoForce.x(), chronoForce.y(), chronoForce.z());
    }

    Torque FrBody_::GetTotalTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
        auto chronoTorque = m_chronoBody->Get_Xtorque();
        return Torque(chronoTorque.x(), chronoTorque.y(), chronoTorque.z());
    }

    // ##CC

    // Nodes

    std::shared_ptr<FrNode_> FrBody_::NewNode() {
        return std::make_shared<FrNode_>(this);
    }

//    std::shared_ptr<FrNode_> FrBody_::NewNode(const frydom::FrFrame_ &nodeFrame) {
//        return std::make_shared<FrNode_>(this, nodeFrame);
//    }
//
//    std::shared_ptr<FrNode_> FrBody_::NewNode(const Position& nodeLocalPosition, const FrRotation_& nodeLocalRotation,
//                                     FRAME_CONVENTION fc) {
//        return NewNode(FrFrame_(nodeLocalPosition, nodeLocalRotation, fc));
//    }
//
//    std::shared_ptr<FrNode_> FrBody_::NewNode(const frydom::Position &nodeLocalPosition, FRAME_CONVENTION fc) {
//        auto NodePositionInBody = nodeLocalPosition;
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(NodePositionInBody);
//        return std::make_shared<FrNode_>(this, NodePositionInBody);
//    }
//
//    std::shared_ptr<FrNode_> FrBody_::NewNode(double x, double y, double z, FRAME_CONVENTION fc) {
//        auto NodePositionInBody = Position(x, y, z);
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(NodePositionInBody);
//        return std::make_shared<FrNode_>(this, NodePositionInBody);
//    }


//    void FrBody_::SetMass(double mass) {
//        m_chronoBody->SetMass(mass);
//    }

    void FrBody_::SetCOG(const Position& bodyPos, FRAME_CONVENTION fc) {
        FrFrame_ cogFrame;
        cogFrame.SetPosition(bodyPos, fc);
        m_chronoBody->UpdateMarkerPositionToCOG(internal::Vector3dToChVector(cogFrame.GetPosition(NWU)));
        m_chronoBody->SetFrame_COG_to_REF(internal::FrFrame2ChFrame(cogFrame));
    }

    Position FrBody_::GetCOG(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_COG_to_REF().GetPos()); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    Position FrBody_::GetPosition(FRAME_CONVENTION fc) const {
        Position refPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_REF_to_abs().GetPos());
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(refPos);
        return refPos;
    }

    FrGeographicCoord FrBody_::GetGeoPosition(FRAME_CONVENTION fc) const {
        return CartToGeo(GetPosition(fc),fc);
    }

    void FrBody_::SetPosition(const Position &worldPos, FRAME_CONVENTION fc) {

        /// This subroutine sets the initial position of a body in world.

        auto bodyFrame = GetFrame();
        bodyFrame.SetPosition(worldPos, fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(bodyFrame));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody_::SetGeoPosition(const FrGeographicCoord& geoCoord) {
        SetPosition(GeoToCart(geoCoord,NWU),NWU);
    }

    FrRotation_ FrBody_::GetRotation() const {
        return FrRotation_(GetQuaternion());
    }

    void FrBody_::SetRotation(const FrRotation_ &rotation) {
        SetRotation(rotation.GetQuaternion());
    }

    FrUnitQuaternion_ FrBody_::GetQuaternion() const {
        return internal::Ch2FrQuaternion(m_chronoBody->GetRot());
    }

    void FrBody_::SetRotation(const FrUnitQuaternion_ &quaternion) {
        Position bodyWorldPos = GetPosition(NWU);
        m_chronoBody->SetRot(internal::Fr2ChQuaternion(quaternion));
        SetPosition(bodyWorldPos, NWU);
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

    FrFrame_ FrBody_::GetFrameAtPoint(const Position& bodyPoint, FRAME_CONVENTION fc) {
        FrFrame_ pointFrame;
        pointFrame.SetPosition(GetPointPositionInWorld(bodyPoint, fc), fc);
        pointFrame.SetRotation(GetQuaternion());
        return pointFrame;
    }

    FrFrame_ FrBody_::GetFrameAtCOG(FRAME_CONVENTION fc) {
        return GetFrameAtPoint(GetCOG(fc), fc);
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


    FrGeographicCoord FrBody_::GetGeoPointPositionInWorld(const Position& bodyPos, FRAME_CONVENTION fc) const {
        return CartToGeo(GetPointPositionInWorld(bodyPos, fc), fc);
    }

    FrGeographicCoord FrBody_::GetGeoPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
        return CartToGeo(GetPointPositionInBody(worldPos, fc), fc);
    }

    FrGeographicCoord FrBody_::GetCOGGeoPosition() const {
        return CartToGeo(GetCOGPositionInWorld(NWU), NWU);
    }


    void FrBody_::SetPositionOfBodyPoint(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc) {
        Position bodyWorldPos = GetPosition(fc);
        Position worldPointPos = GetPointPositionInWorld(bodyPoint, fc);

        Translation translation = worldPos - worldPointPos;
        TranslateInWorld(translation, fc);
    }

    void FrBody_::TranslateInWorld(const Translation &worldTranslation, FRAME_CONVENTION fc) {
        auto refFrame = GetFrame();
        refFrame.SetPosition(refFrame.GetPosition(fc) + worldTranslation, fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(refFrame));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody_::TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInWorld(Translation(x, y, z), fc);
    }

    void FrBody_::TranslateInBody(const Translation &bodyTranslation, FRAME_CONVENTION fc) {
        auto refFrame = GetFrame();
        refFrame.SetPosition(refFrame.GetPosition(fc) + ProjectVectorInWorld<Position>(bodyTranslation, fc), fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(refFrame));
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody_::TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInBody(Translation(x, y, z), fc);
    }

    void FrBody_::Rotate(const FrRotation_ &relRotation) {
        SetRotation(GetRotation() * relRotation);
    }

    void FrBody_::Rotate(const FrUnitQuaternion_ &relQuaternion) {
        SetRotation(GetQuaternion() * relQuaternion);
    }

    void FrBody_::RotateAroundPointInWorld(const FrRotation_& rot, const Position& worldPos, FRAME_CONVENTION fc) {
        RotateAroundPointInWorld(rot.GetQuaternion(), worldPos, fc);
    }

    void FrBody_::RotateAroundPointInBody(const FrRotation_& rot, const Position& bodyPos, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot.GetQuaternion(), bodyPos, fc);
    }

    void FrBody_::RotateAroundPointInWorld(const FrUnitQuaternion_& rot, const Position& worldPos, FRAME_CONVENTION fc) {
        Position bodyPos = GetPointPositionInBody(worldPos, fc);
        Rotate(rot);
        SetPositionOfBodyPoint(bodyPos, worldPos, fc);
    }

    void FrBody_::RotateAroundPointInBody(const FrUnitQuaternion_& rot, const Position& bodyPos, FRAME_CONVENTION fc) {
        Position worldPos = GetPointPositionInWorld(bodyPos, fc);
        Rotate(rot);
        SetPositionOfBodyPoint(bodyPos, worldPos, fc);
    }

    void FrBody_::RotateAroundCOG(const FrRotation_& rot, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot, GetCOG(fc), fc);
    }

    void FrBody_::RotateAroundCOG(const FrUnitQuaternion_& rot, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot, GetCOG(fc), fc);
    }

    void FrBody_::SetGeneralizedVelocityInWorld(const Velocity& worldVel, const AngularVelocity& worldAngVel,
            FRAME_CONVENTION fc) {
        SetGeneralizedVelocityInWorldAtPointInBody(Position(0., 0., 0.), worldVel, worldAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInBody(const Velocity& bodyVel, const AngularVelocity& bodyAngVel,
            FRAME_CONVENTION fc) {
        SetGeneralizedVelocityInBodyAtPointInBody(Position(0., 0., 0.), bodyVel, bodyAngVel, fc);
    }

    Velocity FrBody_::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity bodyVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(bodyVel);
        return bodyVel;
    }

    Velocity FrBody_::GetVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetVelocityInWorld(fc), fc);
    }

    void FrBody_::SetVelocityInWorldNoRotation(const Velocity &worldVel, FRAME_CONVENTION fc) {
        auto worldVelTmp = worldVel;
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(worldVelTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldVelTmp);
        coord.rot.SetNull();
        m_chronoBody->SetCoord_dt(coord);
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody_::SetVelocityInBodyNoRotation(const Velocity &bodyVel, FRAME_CONVENTION fc) {
        SetVelocityInWorldNoRotation(ProjectVectorInWorld(bodyVel, fc), fc);
    }

    Velocity FrBody_::GetCOGVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity cogVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetCoord_dt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(cogVel);
        return cogVel;
    }

    Velocity FrBody_::GetCOGVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetCOGVelocityInWorld(fc), fc);
    }

    void FrBody_::SetAccelerationInWorldNoRotation(const Acceleration &worldAcc, FRAME_CONVENTION fc) {
        auto worldAccTmp = worldAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(worldAccTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldAccTmp);
        coord.rot.SetNull();
        m_chronoBody->SetCoord_dtdt(coord);
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody_::SetAccelerationInBodyNoRotation(const Acceleration &bodyAcc, FRAME_CONVENTION fc) {
        SetAccelerationInWorldNoRotation(ProjectVectorInWorld<Acceleration>(bodyAcc, fc), fc);
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
        m_chronoBody->UpdateAfterMove();
    }

    void FrBody_::SetCOGAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
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
        if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(worldAngAccTmp);
        auto chronoAngAcc = internal::Vector3dToChVector(worldAngAccTmp);
        m_chronoBody->SetWacc_par(chronoAngAcc); // FIXME : dans chrono, l'argument d'entree n'est pas const... -> fix Chrono
        m_chronoBody->UpdateAfterMove();
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
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetVelocityInBodyAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody_::GetVelocityInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        Velocity bodyVel = GetVelocityInBody(fc);
        AngularVelocity bodyAngVel = GetAngularVelocityInBody(fc);
        return bodyVel + bodyAngVel.cross(bodyPoint);
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
        return GetAccelerationInBodyAtPointInBody(GetPointPositionInBody(worldPoint, fc), fc);
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

        Position PG = GetCOG(fc) - bodyPoint;
        Velocity cogVel = bodyVel + bodyAngVel.cross(PG);
        SetVelocityInBodyNoRotation(cogVel, fc);
        SetAngularVelocityInBody(bodyAngVel, fc);
    }

    void FrBody_::SetGeneralizedAccelerationInBodyAtCOG(const Acceleration &bodyAcc, const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {
        SetAccelerationInBodyNoRotation(bodyAcc, fc);
        SetAngularAccelerationInBody(bodyAngAcc, fc);
    }

    void FrBody_::SetGeneralizedAccelerationInWorldAtCOG(const Acceleration &worldAcc, const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {
        SetAccelerationInWorldNoRotation(worldAcc, fc);
        SetAngularAccelerationInWorld(worldAngAcc, fc);
    }

    void FrBody_::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
        geoCoord = CartToGeo(cartPos, fc);
    }
    void FrBody_::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) {
        geoCoord = CartToGeo(cartPos, fc);
    }

    FrGeographicCoord FrBody_::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) const {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->CartToGeo(cartPos, fc);
    }

    FrGeographicCoord FrBody_::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->CartToGeo(cartPos, fc);
    }


    void FrBody_::GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc) const {
        cartPos = GeoToCart(geoCoord, fc);
    }

    void FrBody_::GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc) {
        cartPos = GeoToCart(geoCoord, fc);
    }

    Position FrBody_::GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) const {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->GeoToCart(geoCoord, fc);
    }

    Position FrBody_::GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) {
        return GetSystem()->GetEnvironment()->GetGeographicServices()->GeoToCart(geoCoord, fc);
    }

    void FrBody_::InitializeLockedDOF() {

        // TODO : voir si n'accepte pas de definir des offset sur les ddl bloques...

        // Getting the markers that enter in the link

        // Body marker placed at the current COG body position  // TODO : voir si on se donne d'autres regles que le COG...
        auto bodyNode = NewNode();

        auto cogPositionInWorld= GetCOGPositionInWorld(NWU);
        auto bodyOrientationInWorld = GetQuaternion();

        auto bodyNodeFrameInWorld = FrFrame_(cogPositionInWorld, bodyOrientationInWorld, NWU);
        bodyNode->SetFrameInWorld(bodyNodeFrameInWorld);

        // World Marker placed at the current COG body position
        auto worldNode = GetSystem()->GetWorldBody()->NewNode();
        worldNode->SetFrameInBody(bodyNodeFrameInWorld);

        // Creating the link
        m_DOFLink = std::make_shared<FrLink_>(worldNode, bodyNode, GetSystem());

        // Initializing the link with the DOFMask
        m_DOFLink->InitializeWithBodyDOFMask(m_DOFMask.get());

        // Adding the link to the system
        m_system->AddLink(m_DOFLink);

    }

    FrBodyDOFMask* FrBody_::GetDOFMask() {
        return m_DOFMask.get();
    }



}  // end namespace frydom
