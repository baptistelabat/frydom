//
// Created by frongere on 08/06/17.
//


#include "FrForce.h"

#include "FrBody.h"

#include "FrNode.h"


namespace frydom{

//    void FrForce::GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const {
//        body_force = force;
//        body_torque = moment;
//    }




//    void FrForce::UpdateApplicationPoint() {
//
//        auto my_body = GetBody();
//        auto vmotion = chrono::VNULL;
//
//        if (move_x)
//            vmotion.x() = move_x->Get_y(ChTime);
//        if (move_y)
//            vmotion.y() = move_y->Get_y(ChTime);
//        if (move_z)
//            vmotion.z() = move_z->Get_y(ChTime);
//
//        switch (frame) {
//            case WORLD:
//                vpoint = Vadd(restpos, vmotion);                // Uw
//                vrelpoint = my_body->Point_World2Body(vpoint);  // Uo1 = [A]'(Uw-Xo1)
//                break;
//            case BODY:
//                vrelpoint = Vadd(restpos, vmotion);             // Uo1
//                vpoint = my_body->Point_Body2World(vrelpoint);  // Uw = Xo1+[A]Uo1
//                break;
//        }
//    }
//
//    void FrForce::UpdateChronoForce() {
//
//        double modforce;
//        auto my_body = GetBody();
//        auto vectforce = chrono::VNULL;
//        auto xyzforce = chrono::VNULL;
//
//        modforce = mforce * modula->Get_y(ChTime);
//
//        if (f_x)
//            xyzforce.x() = f_x->Get_y(ChTime);
//        if (f_y)
//            xyzforce.y() = f_y->Get_y(ChTime);
//        if (f_z)
//            xyzforce.z() = f_z->Get_y(ChTime);
//
//        switch (align) {
//            case WORLD_DIR:
//                vreldir = my_body->TransformDirectionParentToLocal(vdir);
//                vectforce = Vmul(vdir, modforce);
//                vectforce = Vadd(vectforce, xyzforce);
//                break;
//            case BODY_DIR:
//                vdir = my_body->TransformDirectionLocalToParent(vreldir);
//                vectforce = Vmul(vdir, modforce);
//                xyzforce = my_body->TransformDirectionLocalToParent(xyzforce);
//                vectforce = Vadd(vectforce, xyzforce);
//                break;
//        }
//
//        force += vectforce;                                           // Fw
//        relforce = my_body->TransformDirectionParentToLocal(force);  // Fo1 = [A]'Fw
//    }

    void FrForce::SetLog() {

        SetLogPrefix();

        //m_log.AddField("time","s","Current time of the simulation",&ChTime);
        m_log.AddField(m_logPrefix + "FX", "N", "Force in x-direction", &force.x());
        m_log.AddField(m_logPrefix + "FY", "N", "Force in y-direction", &force.y());
        m_log.AddField(m_logPrefix + "FZ", "N", "Force in z-direction", &force.z());
        m_log.AddField(m_logPrefix + "MX", "N.m", "Moment along x-direction", &moment.x());
        m_log.AddField(m_logPrefix + "MY", "N.m", "Moment along y-direction", &moment.y());
        m_log.AddField(m_logPrefix + "MZ", "N.m", "Moment along z-direction", &moment.z());

        m_log.AddCSVSerializer();
    }

    void FrForce::InitializeLogs() {

        m_log.Initialize();
        m_log.Send();
    }

    void FrForce::UpdateLogs() {
        m_log.Serialize();
        m_log.Send();
    }





















    /// REFACTORING -------------6>>>>>>>>>>>>>>>>>

    _FrForceBase::_FrForceBase(FrForce_* force) : m_frydomForce(force) {}

    void _FrForceBase::UpdateState() {}


    FrForce_::FrForce_(std::shared_ptr<FrNode_> node) : m_body(node->GetBody()), m_node(node) {
        m_chronoForce = std::make_shared<_FrForceBase>(this);
    }

    std::shared_ptr<chrono::ChForce> FrForce_::GetChronoForce() {
        return m_chronoForce;
    }

    FrOffshoreSystem_* FrForce_::GetSystem() {
        return m_body->GetSystem();
    }

    void FrForce_::SetLocalMomentAtNode(const Moment &momentAtNode) {
        // Moment is the moment at marker point expressed in NWU
        // Should be called at the end of the update procedure with argument moment expressed at node in NWU
        // Warning, force must be already updated into the object...
        // FIXME : avoir une approche plus integree, supprimant la precedence...

        if (!m_body) {
            // TODO : Throw exception

        }

        //        m_node->m_chronoMarker->

        // M_COG = M_marker + COG_marker <cross> force



    }

    void FrForce_::SetAbsMomentAtNode(const Moment &momentAtNode) {
        // TODO
    }

    void FrForce_::SetLocalForce(const Force &force) {
        m_chronoForce->force = internal::Vector3dToChVector(m_body->ProjectOnAbs<Force>(force));
    }

    void FrForce_::SetAbsForce(const Force &force) {
        m_chronoForce->force = internal::Vector3dToChVector(force);
    }

    void FrForce_::SetMaxForceLimit(double fmax) {
        // TODO
    }

    void FrForce_::SetMaxTorqueLimit(double tmax) {
        // TODO
    }

    void FrForce_::SetLimit(bool val) {
        // TODO
    }

    void FrForce_::GetAbsForce(Force &force, FRAME_CONVENTION fc) const {
        // TODO
    }

    Force FrForce_::GetAbsForce(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetAbsForce(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetLocalForce(Force &force, FRAME_CONVENTION fc) const {
        // TODO
    }

    Force FrForce_::GetLocalForce(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetLocalForce(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetAbsMoment(Force &force, FRAME_CONVENTION fc) const {
        // TODO
    }

    Force FrForce_::GetAbsMoment(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetAbsMoment(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetLocalMoment(Force &force, FRAME_CONVENTION fc) const {
        // TODO
    }

    Force FrForce_::GetLocalMoment(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetLocalMoment(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        // TODO
    }

    double FrForce_::GetForceNorm() const {
        // TODO
    }

    double FrForce_::GetTorqueNorm() const {
        // TODO
    }


}  // end namespace frydom