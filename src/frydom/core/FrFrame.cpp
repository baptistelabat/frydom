//
// Created by frongere on 20/09/18.
//

#include "FrFrame.h"

#include "chrono/core/ChMatrixDynamic.h"

#include "FrRotation.h"


namespace frydom {

    FrFrame_::FrFrame_() = default;

    FrFrame_::FrFrame_(const Position &pos, const FrRotation_ &rotation) {
        SetPosition(pos);
        SetRotation(rotation);
    }

    FrFrame_::FrFrame_(const Position &pos, const frydom::FrQuaternion_ &quaternion) {
        SetPosition(pos);
        SetRotation(quaternion);
    }

    FrFrame_& FrFrame_::FrFrame(const FrFrame_ &otherFrame) {
        m_chronoFrame = otherFrame.m_chronoFrame;
        m_frameConvention = otherFrame.m_frameConvention;
    }

    void FrFrame_::SetPosition(double x, double y, double z) {
        m_chronoFrame.SetPos(chrono::ChVector<double>(x, y, z));
    }

    void FrFrame_::SetPosition(Position position) {
        m_chronoFrame.SetPos(internal::Vector3dToChVector(position));
    }

    void FrFrame_::GetPosition(double &x, double &y, double &z) const {
        auto pos = m_chronoFrame.GetPos();
        x = pos.x();
        y = pos.y();
        z = pos.z();
    }

    void FrFrame_::GetPosition(Position &position) const {
        position = GetPosition();
    }

    Position FrFrame_::GetPosition() const {
        return internal::ChVectorToVector3d<Position>(m_chronoFrame.GetPos(), GetFrameConvention());
    }

    void FrFrame_::SetRotation(const FrRotation_ &rotation) {
        SetRotation(rotation.GetQuaternion());
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

    FrRotation_ FrFrame_::GetRotation() const {
        return FrRotation_(GetQuaternion());
    }

    FrQuaternion_ FrFrame_::GetQuaternion() const {
        return internal::Ch2FrQuaternion(m_chronoFrame.GetRot());
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
        FrFrame_ newFrame;
        newFrame.m_chronoFrame = otherFrame.m_chronoFrame >> m_chronoFrame;
        return newFrame;
    }

    void FrFrame_::operator*=(const FrFrame_ &otherFrame) {
        m_chronoFrame >>= otherFrame.m_chronoFrame;
    }

    void FrFrame_::SetX(double x) {
        m_chronoFrame.GetPos()[0] = x;
    }

    void FrFrame_::SetY(double y) {
        m_chronoFrame.GetPos()[1] = y;
    }

    void FrFrame_::SetZ(double z) {
        m_chronoFrame.GetPos()[2] = z;
    }

    double FrFrame_::GetX() const {
        return m_chronoFrame.GetPos()[0];
    }

    double &FrFrame_::GetX() {
        return m_chronoFrame.GetPos()[0];
    }

    double FrFrame_::GetY() const {
        return m_chronoFrame.GetPos()[1];
    }

    double &FrFrame_::GetY() {
        return m_chronoFrame.GetPos()[1];
    }

    double FrFrame_::GetZ() const {
        return m_chronoFrame.GetPos()[2];
    }

    double &FrFrame_::GetZ() {
        return m_chronoFrame.GetPos()[2];
    }

//    FrRotation_ &FrFrame_::GetRotation() {
//        return m_chronoFrame
//    }

    FrFrame_ FrFrame_::GetOtherFrameRelativeTransform_WRT_ThisFrame(const frydom::FrFrame_ &otherFrame) const {
        return this->GetInverse() * otherFrame;
    }

    FrFrame_ FrFrame_::GetThisFrameRelativeTransform_WRT_OtherFrame(const frydom::FrFrame_ &otherFrame) const {
        return GetOtherFrameRelativeTransform_WRT_ThisFrame(otherFrame).GetInverse();
    }


    std::ostream& FrFrame_::cout(std::ostream &os) const {

        double x, y, z;
        GetPosition(x, y, z);

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

    void FrFrame_::RotX_RADIANS(double) {
        // TODO
//        m_chronoFrame.SetRot()
    }

    void FrFrame_::RotX_DEGREES(double angle) {
        // TODO
    }

    void FrFrame_::RotY_RADIANS(double) {
        // TODO
    }

    void FrFrame_::RotY_DEGREES(double angle) {
        // TODO
    }

    void FrFrame_::RotZ_RADIANS(double) {
        // TODO
    }

    void FrFrame_::RotZ_DEGREES(double angle) {
        // TODO
    }

    FrFrame_ &FrFrame_::Inverse() {
        m_chronoFrame.Invert();
        return *this;
    }

    FrFrame_ FrFrame_::GetInverse() const {
        return internal::Ch2FrFrame(m_chronoFrame.GetInverse(), m_frameConvention);
    }

    void FrFrame_::SetRotX_RADIANS(double angle) {
        SetIdentity();
        RotX_RADIANS(angle);
    }

    void FrFrame_::SetRotX_DEGREES(double angle) {
        SetIdentity();
        RotX_DEGREES(angle);
    }

    void FrFrame_::SetRotY_RADIANS(double angle) {
        SetIdentity();
        RotY_RADIANS(angle);
    }

    void FrFrame_::SetRotY_DEGREES(double angle) {
        SetIdentity();
        RotY_DEGREES(angle);
    }

    void FrFrame_::SetRotZ_RADIANS(double angle) {
        SetIdentity();
        RotZ_RADIANS(angle);
    }

    void FrFrame_::SetRotZ_DEGREES(double angle) {
        SetIdentity();
        RotZ_DEGREES(angle);
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

    void FrFrame_::SetFrameConvention(FRAME_CONVENTION frameConvention, bool change) {
        if (m_frameConvention != frameConvention) {
            if (change) {
                SwapAbsFrameConvention();
            }
            m_frameConvention = frameConvention;
        }
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