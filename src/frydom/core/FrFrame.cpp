//
// Created by frongere on 20/09/18.
//

#include "FrFrame.h"

#include "chrono/core/ChMatrixDynamic.h"

#include "FrRotation.h"


namespace frydom {

    FrFrame_::FrFrame_(FRAME_CONVENTION fc) : m_frameConvention(fc), m_chronoFrame() {};

    FrFrame_::FrFrame_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc) : m_frameConvention(fc) {
        SetPosition(pos);
        SetRotation(rotation);
    }

    FrFrame_::FrFrame_(const Position &pos, const frydom::FrQuaternion_ &quaternion, FRAME_CONVENTION fc) : m_frameConvention(fc) {
        SetPosition(pos);
        SetRotation(quaternion);
    }

    FrFrame_& FrFrame_::FrFrame(const FrFrame_ &otherFrame) {
        m_chronoFrame = otherFrame.m_chronoFrame;
        m_frameConvention = otherFrame.m_frameConvention;
    }

    void FrFrame_::SetPosition(double x, double y, double z, FRAME_CONVENTION fc) {

        if (IsNED(fc)) {  // Set In NWU
            y = -y;
            z = -z;
        }

        m_chronoFrame.SetPos(chrono::ChVector<double>(x, y, z));
    }

    void FrFrame_::SetPosition(const Position& position) {
        m_chronoFrame.SetPos(internal::Vector3dToChVector(position));
    }

    void FrFrame_::GetPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const {
        auto pos = m_chronoFrame.GetPos();  // In NWU
        x = pos.x();
        y = pos.y();
        z = pos.z();

        if (IsNED(fc)) {
            y = -y;
            z = -z;
        }
    }

    void FrFrame_::GetPosition(Position &position) const {
        position = GetPosition(m_frameConvention);
    }

    Position FrFrame_::GetPosition(FRAME_CONVENTION fc) const {
        auto pos = internal::ChVectorToVector3d<Position>(m_chronoFrame.GetPos()); // In NWU
        pos.SetFrameConvention(fc, true);
        return pos;
    }

    void FrFrame_::SetX(double x, FRAME_CONVENTION fc) {
        m_chronoFrame.GetPos()[0] = x;
    }

    void FrFrame_::SetY(double y, FRAME_CONVENTION fc) {
        if (IsNED(fc)) y = -y;
        m_chronoFrame.GetPos()[1] = y;
    }

    void FrFrame_::SetZ(double z, FRAME_CONVENTION fc) {
        if (IsNED(fc)) z = -z;
        m_chronoFrame.GetPos()[2] = z;
    }

    double FrFrame_::GetX(FRAME_CONVENTION fc) const {
        return m_chronoFrame.GetPos()[0];
    }

    double &FrFrame_::GetX() {
        return m_chronoFrame.GetPos()[0];
    }

    double FrFrame_::GetY(FRAME_CONVENTION fc) const {
        return -m_chronoFrame.GetPos()[1];
    }

    double &FrFrame_::GetY() {
        return m_chronoFrame.GetPos()[1];
    }

    double FrFrame_::GetZ(FRAME_CONVENTION fc) const {
        return -m_chronoFrame.GetPos()[2];
    }

    double &FrFrame_::GetZ() {
        return m_chronoFrame.GetPos()[2];
    }

    void FrFrame_::SetRotation(const FrRotation_ &rotation) {
        SetRotation(rotation.GetQuaternion(NWU));
    }

    void FrFrame_::SetRotation(const FrQuaternion_ &quaternion) {
        m_chronoFrame.SetRot(internal::Fr2ChQuaternion(quaternion));
    }

    void FrFrame_::SetNoRotation() {
        auto curPos = m_chronoFrame.GetPos();
        m_chronoFrame.SetIdentity();
        m_chronoFrame.SetPos(curPos);
    }

    void FrFrame_::SetNoTranslation() {
        m_chronoFrame.GetPos().SetNull();
    }

    void FrFrame_::SetIdentity() {
        m_chronoFrame.SetIdentity();
    }

    FrRotation_ FrFrame_::GetRotation(FRAME_CONVENTION fc) const {
        return FrRotation_(GetQuaternion(fc));
    }

    FrQuaternion_ FrFrame_::GetQuaternion(FRAME_CONVENTION fc) const {
        auto quat = internal::Ch2FrQuaternion(m_chronoFrame.GetRot());
        quat.SetFrameConvention(fc);
    }

//    FrFrame_ FrFrame_::ApplyToLeft(const FrFrame_ &otherFrame) const {
//        FrFrame_ newFrame;
//        newFrame.m_chronoFrame = m_chronoFrame >> otherFrame.m_chronoFrame;
//        return newFrame;
//    }
//
//    void FrFrame_::ApplyToLeft(const FrFrame_ &otherFrame) {
//        m_chronoFrame >>= otherFrame.m_chronoFrame;
//    }
//
//    FrFrame_ FrFrame_::ApplyToRight(const FrFrame_ &otherFrame) const {
//        return FrFrame_();
//    }
//
//    void FrFrame_::ApplyToRight(const FrFrame_ &otherFrame) {
//
//    }

    FrFrame_ FrFrame_::operator*(const FrFrame_ &otherFrame) const {
        auto newFrame = FrFrame_(m_frameConvention);
        newFrame.m_chronoFrame = otherFrame.m_chronoFrame >> m_chronoFrame;
        return newFrame;
    }

    void FrFrame_::operator*=(const FrFrame_ &otherFrame) {
        m_chronoFrame >>= otherFrame.m_chronoFrame;
    }

    FrFrame_ FrFrame_::GetOtherFrameRelativeTransform_WRT_ThisFrame(const frydom::FrFrame_ &otherFrame, FRAME_CONVENTION fc) const {
        auto relFrame = this->GetInverse() * otherFrame;
        relFrame.SetFrameConvention(fc);
    }

    FrFrame_ FrFrame_::GetThisFrameRelativeTransform_WRT_OtherFrame(const frydom::FrFrame_ &otherFrame, FRAME_CONVENTION fc) const {
        return GetOtherFrameRelativeTransform_WRT_ThisFrame(otherFrame, fc).GetInverse();
    }


    std::ostream& FrFrame_::cout(std::ostream &os) const {

        double x, y, z;
        GetPosition(x, y, z, NWU);

        os << std::endl;
        os << "Frame :\n";
        os << "-------\n";
        os << "Translation (m): ";
        os << "X = " << x;
        os << "; Y = " << y;
        os << "; Z = " << z;
        os << std::endl;
        os << GetRotation();
        os << std::endl;

        return os;

    }

    std::ostream& operator<<(std::ostream& os, const FrFrame_& frame) {
        return frame.cout(os);
    }

    void FrFrame_::RotX_RADIANS(double angle, FRAME_CONVENTION fc) {
        // TODO
//        m_chronoFrame.SetRot()
    }

    void FrFrame_::RotX_DEGREES(double angle, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrFrame_::RotY_RADIANS(double angle, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrFrame_::RotY_DEGREES(double angl, FRAME_CONVENTION fce) {
        // TODO
    }

    void FrFrame_::RotZ_RADIANS(double angle, FRAME_CONVENTION fc) {
        // TODO
    }

    void FrFrame_::RotZ_DEGREES(double angle, FRAME_CONVENTION fc) {
        // TODO
    }



    void FrFrame_::SetRotX_RADIANS(double angle, FRAME_CONVENTION fc) {
        SetIdentity();
        RotX_RADIANS(angle, fc);
    }

    void FrFrame_::SetRotX_DEGREES(double angle, FRAME_CONVENTION fc) {
        SetIdentity();
        RotX_DEGREES(angle, fc);
    }

    void FrFrame_::SetRotY_RADIANS(double angle, FRAME_CONVENTION fc) {
        SetIdentity();
        RotY_RADIANS(angle, fc);
    }

    void FrFrame_::SetRotY_DEGREES(double angle, FRAME_CONVENTION fc) {
        SetIdentity();
        RotY_DEGREES(angle, fc);
    }

    void FrFrame_::SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc) {
        SetIdentity();
        RotZ_RADIANS(angle, fc);
    }

    void FrFrame_::SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc) {
        SetIdentity();
        RotZ_DEGREES(angle, fc);
    }



    FRAME_CONVENTION FrFrame_::GetFrameConvention() const {
        return m_frameConvention;
    }

    FRAME_CONVENTION FrFrame_::SwapAbsFrameConvention() {
        auto quat = m_chronoFrame.GetRot();

        quat.e2() *= -1;
        quat.e3() *= -1;

        m_chronoFrame.SetRot(quat);

        if (m_frameConvention == NWU) {
            m_frameConvention = NED;
        } else {
            m_frameConvention = NWU;
        }

        return m_frameConvention;
    }

    void FrFrame_::SetFrameConvention(FRAME_CONVENTION fc) {
        m_frameConvention = fc;
    }

    void FrFrame_::SetNWU() {
        SetFrameConvention(NWU, true);
    }

    void FrFrame_::SetNED() {
        SetFrameConvention(NED, true);
    }

    void FrFrame_::GetGeographicPosition(double &latitude, double &longitude, double &height) const {
        // TODO
    }

    double FrFrame_::GetLatitude() const {
        // TODO
    }

    double FrFrame_::GetLongitude() const {
        // TODO
    }

    double FrFrame_::GetGeographicHeight() const {
        // TODO
    }


    FrFrame_ &FrFrame_::Inverse() {
        m_chronoFrame.Invert();
        return *this;
    }

    FrFrame_ FrFrame_::GetInverse() const {
        return internal::Ch2FrFrame(m_chronoFrame.GetInverse(), m_frameConvention);
    }




//    std::shared_ptr<FrFrame_> FrFrame_::NewRelFrame(Vector3d pos, FrRotation_ rot) const {
//        auto newFrame = std::make_shared<FrFrame_>();
//        // TODO : utiliser la position et la rotation
//
//
//
//
//    }
//
//    std::shared_ptr<FrFrame_> FrFrame_::NewRelFrame(Vector3d pos) const {
//        return std::make_shared<FrFrame_>();
//    }
//
//    std::shared_ptr<FrFrame_> FrFrame_::NewRelFrame(FrRotation_ rot) const {
//        return std::make_shared<FrFrame_>();
//    }


}  // end namespace frydom