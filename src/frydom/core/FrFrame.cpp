//
// Created by frongere on 20/09/18.
//

#include "FrFrame.h"

#include "chrono/core/ChMatrixDynamic.h"

#include "FrRotation.h"


namespace frydom {

    namespace internal {

        FrFrame_ Ch2FrFrame(const chrono::ChFrame<double>& chFrame) {
            auto pos = ChVectorToVector3d(chFrame.GetPos());
            auto quat = Ch2FrQuaternion(chFrame.GetRot());
            return FrFrame_(pos, quat);
        }

        chrono::ChFrame<double> Fr2ChFrame(const FrFrame_& frFrame) {
            auto pos = Vector3dToChVector(frFrame.GetPosition());
            auto quat = Fr2ChQuaternion(frFrame.GetQuaternion());
            chrono::ChFrame<double>(pos, quat);
        }

    }


    FrFrame_::FrFrame_() = default;

    FrFrame_::FrFrame_(const Vector3d &pos, const FrRotation_ &rotation) {
        SetPosition(pos);
        SetRotation(rotation);
    }

    FrFrame_::FrFrame_(const Vector3d &pos, const frydom::FrQuaternion_ &quaternion) {
        SetPosition(pos);
        SetRotation(quaternion);
    }

    FrFrame_& FrFrame_::FrFrame(const FrFrame_ &otherFrame) {
        m_chronoFrame = otherFrame.m_chronoFrame;
    }

    void FrFrame_::SetPosition(double x, double y, double z) {
        m_chronoFrame.SetPos(chrono::ChVector<double>(x, y, z));
    }

    void FrFrame_::SetPosition(Vector3d position) {
        m_chronoFrame.SetPos(internal::Vector3dToChVector(position));
    }

    void FrFrame_::GetPosition(double &x, double &y, double &z) const {
        auto pos = m_chronoFrame.GetPos();
        x = pos.x();
        y = pos.y();
        z = pos.z();
    }

    void FrFrame_::GetPosition(Vector3d &position) const {
        position = GetPosition();
    }

    Vector3d FrFrame_::GetPosition() const {
        return internal::ChVectorToVector3d(m_chronoFrame.GetPos());
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
        newFrame.m_chronoFrame = m_chronoFrame >> otherFrame.m_chronoFrame;
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

    FrFrame_ FrFrame_::GetOtherFrameRelativeTransform(const frydom::FrFrame_ &otherFrame) {
        auto tFA = this->GetInverse();
        auto tAO = otherFrame;

        auto tFO = tFA * tAO;

        return this->GetInverse() * otherFrame;
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

    }

    std::ostream& operator<<(std::ostream& os, const FrFrame_& frame) {
        return frame.cout(os);
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