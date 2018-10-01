//
// Created by frongere on 20/09/18.
//

#include "FrFrame.h"

#include "chrono/core/ChMatrixDynamic.h"

#include "FrRotation.h"


namespace frydom {


    FrFrame_::FrFrame_() : m_chronoFrame() {};  // OK

    FrFrame_::FrFrame_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc) {  // OK
        SetPosition(pos, fc);
        SetRotation(rotation);
    }

    FrFrame_::FrFrame_(const Position &pos, const frydom::FrQuaternion_ &quaternion, FRAME_CONVENTION fc) {  // OK
        SetPosition(pos, fc);
        SetRotation(quaternion);
    }

    FrFrame_& FrFrame_::FrFrame(const FrFrame_ &otherFrame) {  // OK
        m_chronoFrame = otherFrame.m_chronoFrame;
    }

    void FrFrame_::SetPosition(double x, double y, double z, FRAME_CONVENTION fc) {  // OK

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z); // Express In NWU
        m_chronoFrame.SetPos(chrono::ChVector<double>(x, y, z));
    }

    void FrFrame_::SetPosition(const Position& position, FRAME_CONVENTION fc) {  // OK
        auto posTmp = position;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(posTmp);
        m_chronoFrame.SetPos(internal::Vector3dToChVector(posTmp));
    }

    void FrFrame_::GetPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const {  // OK
        auto pos = m_chronoFrame.GetPos();  // In NWU
        x = pos.x();
        y = pos.y();
        z = pos.z();

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);
    }

    void FrFrame_::GetPosition(Position &position, FRAME_CONVENTION fc) const {  // OK
        position = GetPosition(fc);
    }

    Position FrFrame_::GetPosition(FRAME_CONVENTION fc) const {  // OK
        auto pos = internal::ChVectorToVector3d<Position>(m_chronoFrame.GetPos()); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(pos);
        return pos;
    }

    void FrFrame_::SetX(double x, FRAME_CONVENTION fc) {  // OK
        m_chronoFrame.GetPos()[0] = x;
    }

    void FrFrame_::SetY(double y, FRAME_CONVENTION fc) {  // OK
        if (IsNED(fc)) y = -y;
        m_chronoFrame.GetPos()[1] = y;
    }

    void FrFrame_::SetZ(double z, FRAME_CONVENTION fc) {  // OK
        if (IsNED(fc)) z = -z;
        m_chronoFrame.GetPos()[2] = z;
    }

    double FrFrame_::GetX(FRAME_CONVENTION fc) const {  // OK
        return m_chronoFrame.GetPos()[0];
    }

    double FrFrame_::GetY(FRAME_CONVENTION fc) const {  // OK
        double y = m_chronoFrame.GetPos()[1];
        if (IsNED(fc)) y = -y;
        return y;
    }

    double FrFrame_::GetZ(FRAME_CONVENTION fc) const {  // OK
        double z = m_chronoFrame.GetPos()[2];
        if (IsNED(fc)) z = -z;
        return z;
    }

    void FrFrame_::SetRotation(const FrRotation_ &rotation) {  // OK
        SetRotation(rotation.GetQuaternion());
    }

    void FrFrame_::SetRotation(const FrQuaternion_ &quaternion) {  // OK
        m_chronoFrame.SetRot(internal::Fr2ChQuaternion(quaternion));
    }

    void FrFrame_::SetNoRotation() {  // OK
        auto curPos = m_chronoFrame.GetPos();
        m_chronoFrame.SetIdentity();
        m_chronoFrame.SetPos(curPos);
    }

    void FrFrame_::SetNoTranslation() {  // OK
        m_chronoFrame.GetPos().SetNull();
    }

    void FrFrame_::SetIdentity() {  // OK
        m_chronoFrame.SetIdentity();
    }

    FrRotation_ FrFrame_::GetRotation(FRAME_CONVENTION fc) const {  // OK
        return FrRotation_(GetQuaternion(fc));
    }

    FrQuaternion_ FrFrame_::GetQuaternion(FRAME_CONVENTION fc) const {  // OK
        return internal::Ch2FrQuaternion(m_chronoFrame.GetRot());  // In NWU
    }

    FrFrame_ FrFrame_::operator*(const FrFrame_ &otherFrame) const {
        auto newFrame = FrFrame_();
        newFrame.m_chronoFrame = otherFrame.m_chronoFrame >> m_chronoFrame;  // TODO : verifier !!
        return newFrame;
    }

    void FrFrame_::operator*=(const FrFrame_ &otherFrame) {
        m_chronoFrame >>= otherFrame.m_chronoFrame;  // TODO : verifier
    }

    FrFrame_ FrFrame_::GetOtherFrameRelativeTransform_WRT_ThisFrame(const frydom::FrFrame_ &otherFrame, FRAME_CONVENTION fc) const {  // OK
        return this->GetInverse() * otherFrame;
    }

    FrFrame_ FrFrame_::GetThisFrameRelativeTransform_WRT_OtherFrame(const frydom::FrFrame_ &otherFrame, FRAME_CONVENTION fc) const {  // OK
        return GetOtherFrameRelativeTransform_WRT_ThisFrame(otherFrame, fc).GetInverse();
    }

    std::ostream& FrFrame_::cout(std::ostream &os) const {  // OK

        double x, y, z;
        GetPosition(x, y, z, NWU);

        os << std::endl;
        os << "Frame :\n";
        os << "-------\n";
        os << "Translation (m, In NWU): ";
        os << "X = " << x;
        os << "; Y = " << y;
        os << "; Z = " << z;
        os << std::endl;
        os << GetRotation(NWU);
        os << std::endl;

        return os;

    }

    std::ostream& operator<<(std::ostream& os, const FrFrame_& frame) {  // OK
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



    void FrFrame_::SetRotX_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotX_RADIANS(angle, fc);
    }

    void FrFrame_::SetRotX_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotX_DEGREES(angle, fc);
    }

    void FrFrame_::SetRotY_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotY_RADIANS(angle, fc);
    }

    void FrFrame_::SetRotY_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotY_DEGREES(angle, fc);
    }

    void FrFrame_::SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotZ_RADIANS(angle, fc);
    }

    void FrFrame_::SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotZ_DEGREES(angle, fc);
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

    FrFrame_ &FrFrame_::Inverse() {  // OK
        m_chronoFrame.Invert();
        return *this;
    }

    FrFrame_ FrFrame_::GetInverse() const {  // OK
        return internal::Ch2FrFrame(m_chronoFrame.GetInverse());
    }


}  // end namespace frydom