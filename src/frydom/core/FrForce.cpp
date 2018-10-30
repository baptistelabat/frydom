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

        if (m_logPrefix == "") { SetLogPrefix(); }  // Set default log prefix if not defined

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

    void _FrForceBase::UpdateState() {

        // Calling the FRyDoM interface for Update
        m_frydomForce->Update(ChTime);

        // Limitation of the force and torque
        if (m_frydomForce->GetLimit()) {

            double limit = m_frydomForce->GetMaxForceLimit();
            double magn = force.Length();
            if (magn > limit) {
                force *= limit / magn;
            }

            limit = m_frydomForce->GetMaxTorqueLimit();
            magn = m_torque.Length();
            if (magn > limit) {
                m_torque *= limit / magn;
            }
        }
    }

    void _FrForceBase::GetBodyForceTorque(chrono::ChVector<double>& body_force,
                                          chrono::ChVector<double>& body_torque) const {
        body_force  = force;    // In absolute coordinates
        body_torque = m_torque; // In body coordinates expressed at COG
    }

    void _FrForceBase::GetAbsForceNWU(Force &body_force) const {
        body_force = internal::ChVectorToVector3d<Force>(force);
    }

    void _FrForceBase::GetLocalTorqueNWU(Moment &body_torque) const {
        body_torque = internal::ChVectorToVector3d<Moment>(m_torque);
    }

    void _FrForceBase::SetAbsForceNWU(const Force &body_force) {
        force = internal::Vector3dToChVector(body_force);
    }

    void _FrForceBase::SetLocalTorqueNWU(const Moment &body_torque) {
        m_torque = internal::Vector3dToChVector(body_torque);
    }






    // FrForce_ methods implementations

    FrForce_::FrForce_(std::shared_ptr<FrNode_> node) : m_body(node->GetBody()), m_node(node) {
        m_chronoForce = std::make_shared<_FrForceBase>(this);
    }

    std::shared_ptr<chrono::ChForce> FrForce_::GetChronoForce() {
        return m_chronoForce;
    }

    FrOffshoreSystem_* FrForce_::GetSystem() {
        return m_body->GetSystem();
    }


    void FrForce_::SetMaxForceLimit(double fmax) {
        m_forceLimit = fmax;
    }

    double FrForce_::GetMaxForceLimit() const {
        return m_forceLimit;
    }

    void FrForce_::SetMaxTorqueLimit(double tmax) {
        m_torqueLimit = tmax;
    }

    double FrForce_::GetMaxTorqueLimit() const {
        return m_torqueLimit;
    }

    void FrForce_::SetLimit(bool val) {
        if (val) {
            m_forceLimit = 1e8;
            m_torqueLimit = 1e8;
        }
        m_limitForce = val;
    }

    bool FrForce_::GetLimit() const {
        return m_limitForce;
    }

    void FrForce_::GetAbsForce(Force &force, FRAME_CONVENTION fc) const {
        m_chronoForce->GetAbsForceNWU(force);  // NWU

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Force>(force);
        }
    }

    Force FrForce_::GetAbsForce(FRAME_CONVENTION fc) const {
        Force force;
        GetAbsForce(force, fc);
        return force;
    }

    void FrForce_::GetAbsForce(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        auto force = GetAbsForce(fc);
        fx = force[0];
        fy = force[1];
        fz = force[2];
    }

    void FrForce_::GetLocalForce(Force &force, FRAME_CONVENTION fc) const {
        GetAbsForce(force, fc);
        m_body->ProjectAbsVectorInBodyCoords<Force>(force, fc);
    }

    Force FrForce_::GetLocalForce(FRAME_CONVENTION fc) const {
        Force force;
        GetLocalForce(force, fc);
        return force;
    }

    void FrForce_::GetLocalForce(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        auto force = GetLocalForce(fc);
        fx = force[0];
        fx = force[1];
        fx = force[2];
    }

    void FrForce_::GetAbsTorqueAtCOG(Moment &torque, FRAME_CONVENTION fc) const {
        // TODO
    }

    Moment FrForce_::GetAbsTorqueAtCOG(FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetAbsTorqueAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrForce_::GetLocalTorqueAtCOG(Moment &torque, FRAME_CONVENTION fc) const {
        m_chronoForce->GetLocalTorqueNWU(torque);

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Moment>(torque);
        }
    }

    Moment FrForce_::GetLocalTorqueAtCOG(FRAME_CONVENTION fc) const {
        Moment torque;
        GetLocalTorqueAtCOG(torque, fc);
    }

    void FrForce_::GetLocalTorqueAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
        // TODO
    }

    double FrForce_::GetForceNorm() const {
        // TODO
    }

    double FrForce_::GetTorqueNormAtCOG() const {
        // TODO
    }

    // =================================================================================================================
    // Protected methods implementations
    // =================================================================================================================

    void FrForce_::SetAbsForce(const Force &force, FRAME_CONVENTION fc) {
        auto forceTmp = force;
        if (IsNED(fc)) {
            internal::SwapFrameConvention<Force>(forceTmp);  // In NWU
        }

        m_chronoForce->SetAbsForceNWU(forceTmp);
    }

    void FrForce_::SetAbsForceOnLocalPoint(const Force &force, const Position &relPos, FRAME_CONVENTION fc) {
        SetAbsForce(force, fc);

        // Calculating the moment created by the force applied at point relPos
        Position GP = relPos - m_body->GetCOGLocalPosition(fc); // In body coordinates following the fc convention

        Moment body_torque = GP.cross(m_body->ProjectAbsVectorInBodyCoords<Force>(force, fc));

        SetLocalTorqueAtCOG(body_torque, fc);
    }

    void FrForce_::SetAbsForceOnAbsPoint(const Force &force, const Position &absPos, FRAME_CONVENTION fc) {
        // Getting the local position of the point
        Position relPos = m_body->ProjectAbsVectorInBodyCoords<Position>(absPos - m_body->GetAbsPosition(fc), fc);
        SetAbsForceOnLocalPoint(force, relPos, fc);
    }

    void FrForce_::SetLocalForce(const Force &force, FRAME_CONVENTION fc) {
        SetAbsForce(m_body->ProjectBodyVectorInAbsCoords<Force>(force, fc), fc);
    }

    void FrForce_::SetLocalForceOnLocalPoint(const Force& force, const Position& relPos, FRAME_CONVENTION fc) {
        SetAbsForceOnLocalPoint(m_body->ProjectBodyVectorInAbsCoords<Force>(force, fc), relPos, fc);
    }

    void FrForce_::SetLocalForceOnAbsPoint(const Force& force, const Position& absPos, FRAME_CONVENTION fc) {
        SetAbsForceOnAbsPoint(m_body->ProjectBodyVectorInAbsCoords<Force>(force, fc), absPos, fc);
    }

    void FrForce_::SetAbsTorqueAtCOG(const Moment& torque, FRAME_CONVENTION fc) {
        SetLocalTorqueAtCOG(m_body->ProjectAbsVectorInBodyCoords<Moment>(torque, fc), fc);
    }

    void FrForce_::SetLocalTorqueAtCOG(const Moment& torque, FRAME_CONVENTION fc) {
        auto torqueTmp = torque;
        if (IsNED(fc)) {
            internal::SwapFrameConvention<Moment>(torqueTmp);  // In NWU
        }

        m_chronoForce->SetLocalTorqueNWU(torqueTmp);
    }


    void FrForce_::SetAbsForceTorqueAtCOG(const Force& force, const Moment& torque, FRAME_CONVENTION fc) {
        SetAbsForce(force, fc);
        SetAbsTorqueAtCOG(torque, fc);
    }

    void FrForce_::SetLocalForceTorqueAtCOG(const Force& force, const Moment& torque, FRAME_CONVENTION fc) {
        SetLocalForce(force, fc);
        SetLocalTorqueAtCOG(torque, fc);
    }

    void FrForce_::SetAbsForceTorqueAtLocalPoint(const Force& force, const Moment& torque, const Position& relPos, FRAME_CONVENTION fc) {
        SetAbsForceOnLocalPoint(force, relPos, fc);
        SetLocalTorqueAtCOG(GetLocalTorqueAtCOG(fc) + m_body->ProjectAbsVectorInBodyCoords<Moment>(torque, fc), fc);
    }

    void FrForce_::SetAbsForceTorqueAtAbsPoint(const Force& force, const Moment& torque, const Position& absPos, FRAME_CONVENTION fc) {
        SetAbsForceOnAbsPoint(force, absPos, fc);
        SetLocalTorqueAtCOG(GetLocalTorqueAtCOG(fc) + m_body->ProjectAbsVectorInBodyCoords<Moment>(torque, fc), fc);
    }

    void FrForce_::SetLocalForceTorqueAtLocalPoint(const Force& force, const Moment& torque, const Position& relPos, FRAME_CONVENTION fc) {
        SetLocalForceOnLocalPoint(force, relPos, fc);
        SetLocalTorqueAtCOG(GetLocalTorqueAtCOG(fc) + torque, fc);
    }

    void FrForce_::SetLocalForceTorqueAtAbsPoint(const Force& force, const Moment& torque, const Position& absPos, FRAME_CONVENTION fc) {
        SetLocalForceOnAbsPoint(force, absPos, fc);
        SetLocalTorqueAtCOG(GetLocalTorqueAtCOG(fc) + torque, fc);
    }

}  // end namespace frydom
