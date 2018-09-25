//
// Created by frongere on 21/06/17.
//

#include "FrBody.h"
#include "FrNode.h"
#include "FrRotation.h"

#include "chrono/assets/ChTriangleMeshShape.h"

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



    _FrBodyBase::_FrBodyBase() : chrono::ChBodyAuxRef() {}


    FrBody_::FrBody_() {
        m_chronoBody = std::make_shared<_FrBodyBase>();
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

    void FrBody_::SetCOGPositionWRTLocalFrame(double x, double y, double z) {

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


    }

    void FrBody_::SetCOGFrameOrientationWRTLocalFrame(FrRotation_ rotation) {

    }

//    void FrBody_::SetCOGFrameWRTLocalFrame(FrTransform_ transform) {
//
//    }

    void FrBody_::SetLocalFrameAbsPosition(double x, double y, double z) {

    }

//    void FrBody_::SetLocalFrameWRTAbsFrame(FrTransform_ transform) {
//
//    }

    void FrBody_::SetCOGFrameAbsPosition(double x, double y, double z) {

    }

//    void FrBody_::SetCOGFrameWRTAbsFrame(FrTransform_ transform) {
//
//    }


}  // end namespace frydom