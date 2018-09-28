//
// Created by frongere on 20/09/18.
//

#include "FrRotation.h"

#include "FrEulerAngles.h"

#include "FrGeographic.h"


namespace frydom {


    // FrQuaternion_

    FrQuaternion_::FrQuaternion_(FRAME_CONVENTION fc) : m_frameConvention(fc), m_chronoQuaternion() {}

    FrQuaternion_::FrQuaternion_(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc) : m_frameConvention(fc){
        if (IsNED(fc)) {
            q2 = -q2;
            q3 = -q3;
        }
        m_chronoQuaternion.Set(q0, q1, q2, q3);
    }

    FrQuaternion_::FrQuaternion_(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) : m_frameConvention(fc) {
        Set(axis, angleRAD);
    }

    FrQuaternion_::FrQuaternion_(const FrQuaternion_ &other) = default;

    void FrQuaternion_::Set(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc) {
        if (IsNED(fc)) {
            q2 = -q2;
            q3 = -q3;
        }
        m_chronoQuaternion.Set(q0, q1, q2, q3);
    }

    void FrQuaternion_::Set(const FrQuaternion_ &quaternion) {  // Remain with the current convention
        m_chronoQuaternion.Set(quaternion.m_chronoQuaternion);
    }

    void FrQuaternion_::Set(const Direction& axis, double angleRAD) {
        auto axisTmp = axis;
        axisTmp.SetNWU(); // Chrono objects always in NWU convention
        m_chronoQuaternion = chrono::Q_from_AngAxis(angleRAD, internal::Vector3dToChVector(axisTmp));
    }

    void FrQuaternion_::SetNull() {
        m_chronoQuaternion.SetNull();
    }

    void FrQuaternion_::Normalize() {
        m_chronoQuaternion.Normalize();
    }

    double FrQuaternion_::Norm() const {
        return m_chronoQuaternion.Length();
    }

//    void FrQuaternion_::SetScalar(double s) {
//        m_chronoQuaternion.SetScalar(s);
//    }
//
//    void FrQuaternion_::SetReal(const Direction &v) {
//        auto vTmp = v;
//
//        if (!HasSameConvention<Direction>(v)) vTmp.SwapFrameConvention();
//
//        m_chronoQuaternion.SetVector(internal::Vector3dToChVector(vTmp));
//    }

//    double FrQuaternion_::GetScalar() const {
//        return m_chronoQuaternion.e0();
//    }
    bool FrQuaternion_::IsRotation() const {
        return (mathutils::IsClose<double>(Norm(), 1.));
    }

    Direction FrQuaternion_::GetDirection(FRAME_CONVENTION fc) const {
        auto direction = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetVector()); // In NWU
        if (IsNED(fc)) direction.SwapFrameConvention();
        return direction;
    }

    void FrQuaternion_::Get(double &q0, double &q1, double &q2, double &q3, FRAME_CONVENTION fc) const {
        q0 = m_chronoQuaternion[0];
        q1 = m_chronoQuaternion[1];
        q2 = m_chronoQuaternion[2];
        q3 = m_chronoQuaternion[3];  // In NWU
        if (IsNED(fc)) {
            q2 = - q2;
            q3 = - q3;
        }
    }

    void FrQuaternion_::Get(Direction& axis, double& angleRAD, FRAME_CONVENTION fc) const {
        chrono::ChVector<double> chronoAxis;
        chrono::Q_to_AngAxis(m_chronoQuaternion, angleRAD, chronoAxis); // In NWU
        axis = internal::ChVectorToVector3d<Direction>(chronoAxis); // In NWU
        axis.SetFrameConvention(fc, true);
    }

    Direction FrQuaternion_::GetXAxis(FRAME_CONVENTION fc) const {
        auto axis = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetXaxis());
        axis.SetFrameConvention(fc, true);
        return axis;
    }

    Direction FrQuaternion_::GetYAxis(FRAME_CONVENTION fc) const {
        auto axis = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetYaxis());
        axis.SetFrameConvention(fc, true);
        return axis;
    }

    Direction FrQuaternion_::GetZAxis(FRAME_CONVENTION fc) const {
        auto axis = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetZaxis());
        axis.SetFrameConvention(fc, true);
        return axis;
    }

    FRAME_CONVENTION FrQuaternion_::GetFrameConvention() const { return m_frameConvention; }

    FRAME_CONVENTION FrQuaternion_::SwapAbsFrameConvention() {

        if (IsNWU(m_frameConvention)) {
            m_frameConvention = NED;
        } else {
            m_frameConvention = NWU;
        }

        return m_frameConvention;
    }

    void FrQuaternion_::SetFrameConvention(FRAME_CONVENTION fc) {
        m_frameConvention = fc;
    }

    void FrQuaternion_::SetNWU() {
        m_frameConvention = NWU;
    }

    void FrQuaternion_::SetNED() {
        m_frameConvention = NED;
    }

    bool FrQuaternion_::HasSameConvention(const FrQuaternion_& other) const {
        return (m_frameConvention == other.m_frameConvention);
    }

    bool FrQuaternion_::HasSameConvention(FRAME_CONVENTION fc) const {
        return (m_frameConvention == fc);
    }

//    bool FrQuaternion_::IsAbsolute() const {
//        return IsAbsoluteConvention(m_frameConvention);
//    }
//
//    bool FrQuaternion_::IsRelative() const {
//        return IsRelativeConvention(m_frameConvention);
//    }

    FrQuaternion_ &FrQuaternion_::operator=(const FrQuaternion_ &other) { // We keep the current frame convention
        m_chronoQuaternion = other.m_chronoQuaternion;
    }

//    FrQuaternion_ FrQuaternion_::operator+() const {
//        return *this;
//    }
//
//    FrQuaternion_ FrQuaternion_::operator-() const {
//        return internal::Ch2FrQuaternion(-m_chronoQuaternion, m_frameConvention);
//    }

    FrQuaternion_ FrQuaternion_::operator*(const FrQuaternion_ &other) const {
        auto newQuat = FrQuaternion_(*this);
        newQuat *= other;
        return newQuat;
    }

    FrQuaternion_ &FrQuaternion_::operator*=(const FrQuaternion_ &other) {
        m_chronoQuaternion *= other.m_chronoQuaternion;
    }

    const chrono::ChQuaternion<double>& FrQuaternion_::GetChronoQuaternion() const {
        return m_chronoQuaternion;
    }

    FrQuaternion_& FrQuaternion_::Inverse() {
        m_chronoQuaternion.Conjugate(); // TODO : verifier
        return *this;
    }

    FrQuaternion_ FrQuaternion_::Inverse() const {
        return FrQuaternion_(*this).Inverse();  // TODO verifier
    }


//    // FrRotationMatrix
//
//    FrRotationMatrix_::FrRotationMatrix_() {
//        m_matrix.SetIdentity();
//    }
//
//    FrRotationMatrix_::FrRotationMatrix_(const frydom::FrRotationMatrix_ &other) : m_matrix(other.GetChMatrix()) {}
//
//    chrono::ChMatrix33<double> FrRotationMatrix_::GetChMatrix() const {
//        return m_matrix;
//    }



    // FrRotation_

    FrRotation_::FrRotation_(FRAME_CONVENTION fc) : m_frQuaternion(fc) {}

    FrRotation_::FrRotation_(FrQuaternion_ quaternion) : m_frQuaternion(quaternion) {}

    FrRotation_::FrRotation_(const Direction& axis, double angleRAD) : m_frQuaternion(axis.GetFrameConvention()) {
        SetAxisAngle(axis, angleRAD);
    }

    void FrRotation_::SetNull() {
        m_frQuaternion.SetNull();
    }

    void FrRotation_::Set(const FrQuaternion_& quat) {
        m_frQuaternion.Set(quat);
    }

    FrQuaternion_ &FrRotation_::GetQuaternion() {
        return m_frQuaternion;
    }

    FrQuaternion_ FrRotation_::GetQuaternion(FRAME_CONVENTION fc) const {
        auto quat = m_frQuaternion;
        quat.SetFrameConvention(fc);
        return quat;
    }

    // We keep the frame convention of the rotation
    void FrRotation_::SetAxisAngle(const Direction &axis, double angleRAD) {
        m_frQuaternion.Set(axis, angleRAD);
    }

    void FrRotation_::GetAxisAngle(Direction &axis, double angleRAD, FRAME_CONVENTION fc) {
        m_frQuaternion.Get(axis, angleRAD, fc);
    }

    void FrRotation_::GetAxis(Direction &axis, FRAME_CONVENTION fc) {
        double angle=0.;
        GetAxisAngle(axis, angle, fc);
    }

    void FrRotation_::GetAngle(double &angle) {
        auto curConvention = GetFrameConvention();
        auto axis = Direction(curConvention);
        GetAxisAngle(axis, angle, curConvention);
    }
//
//    void FrRotation_::SetRotationMatrix(const FrRotationMatrix_ &mat) {
//        m_frQuaternion = internal::Ch2FrQuaternion(internal::mat_to_quat(mat.GetChMatrix()));
//    }
//
//    FrRotationMatrix_ FrRotation_::GetRotationMatrix() const {
//        FrRotationMatrix_ matrix;
//        matrix.m_matrix = internal::quat_to_mat(m_frQuaternion.GetChronoQuaternion());
//        return matrix;
//    }





    void FrRotation_::SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {

        if (IsNED(fc)) {  // FIXME : la convention n'est valable que pour une sequence se terminant par yz...
            // Put in NWU convention
            theta = -theta;
            psi   = -psi;
        }

        m_frQuaternion.Set(internal::Ch2FrQuaternion(internal::euler_to_quat(phi, theta, psi, seq)));

    }

    void FrRotation_::SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
        SetEulerAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, seq, fc);
    }

    void FrRotation_::SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc) {
        SetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
    }

    void FrRotation_::SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc) {
        SetCardanAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, fc);
    }

    void FrRotation_::GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {

        auto vec = internal::quat_to_euler(m_frQuaternion.GetChronoQuaternion(), seq);  // In NWU
        phi   = vec[0];
        theta = vec[1];
        psi   = vec[2];

        if (IsNED(fc)) {  // FIXME : la convention n'est valable que pour une sequence se terminant par yz...
            theta = -theta;
            psi   = -psi;
        }
    }

    void FrRotation_::GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {
        GetEulerAngles_RADIANS(phi, theta, psi, seq, fc);
        phi   *= RAD2DEG;
        theta *= RAD2DEG;
        psi   *= RAD2DEG;
    }

    void FrRotation_::GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
        GetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
    }

    void FrRotation_::GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
        GetEulerAngles_DEGREES(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
    }

    void FrRotation_::GetFixedAxisAngles_RADIANS(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const {
        // TODO
    }

    void FrRotation_::GetFixedAxisAngles_DEGREES(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const {
        // TODO
    }

    FrRotation_& FrRotation_::operator=(const FrRotation_ &other) {
        m_frQuaternion = other.m_frQuaternion;
    }

    FrRotation_ FrRotation_::operator*(const FrRotation_ &other) const {
        auto newRotation = FrRotation_(GetFrameConvention());
        newRotation.Set(m_frQuaternion * other.m_frQuaternion);
        return newRotation;
    }

    FrRotation_ &FrRotation_::operator*=(const FrRotation_ &other) {
        m_frQuaternion *= other.m_frQuaternion;
    }

    Position FrRotation_::Rotate(const Position &vector) {
        return m_frQuaternion.Rotate(vector);
    }

    FrRotation_& FrRotation_::RotAxisAngle_RADIANS(const Direction &axis, double angle) {
        *this *= FrRotation_(axis, angle);
        return *this;
    }

    FrRotation_& FrRotation_::RotAxisAngle_DEGREES(const Direction &axis, double angle) {
        RotAxisAngle_RADIANS(axis, angle*DEG2RAD);
    }

    FrRotation_& FrRotation_::RotX_RADIANS(double angle, FRAME_CONVENTION fc) {
        RotAxisAngle_RADIANS(Direction(1., 0., 0., fc), angle);
    }

    FrRotation_& FrRotation_::RotX_DEGREES(double angle, FRAME_CONVENTION fc) {
        RotX_RADIANS(angle*DEG2RAD, fc);
    }

    FrRotation_& FrRotation_::RotY_RADIANS(double angle, FRAME_CONVENTION fc) {
        RotAxisAngle_RADIANS(Direction(0., 1., 0., fc), angle);
    }

    FrRotation_& FrRotation_::RotY_DEGREES(double angle, FRAME_CONVENTION fc) {
        RotY_RADIANS(angle*DEG2RAD, fc);
    }

    FrRotation_& FrRotation_::RotZ_RADIANS(double angle, FRAME_CONVENTION fc) {
        RotAxisAngle_RADIANS(Direction(0., 0., 1., fc), angle);
    }

    FrRotation_& FrRotation_::RotZ_DEGREES(double angle, FRAME_CONVENTION fc) {
        RotZ_RADIANS(angle*DEG2RAD, fc);
    }

    Direction FrRotation_::GetXAxis(FRAME_CONVENTION fc) const {
        return m_frQuaternion.GetXAxis(fc);
    }

    Direction FrRotation_::GetYAxis(FRAME_CONVENTION fc) const {
        return m_frQuaternion.GetYAxis(fc);
    }

    Direction FrRotation_::GetZAxis(FRAME_CONVENTION fc) const {
        return m_frQuaternion.GetZAxis(fc);
    }

    std::ostream& FrRotation_::cout(std::ostream &os) const {

        double phi, theta, psi;
        GetCardanAngles_DEGREES(phi, theta, psi, NWU);

        os << std::endl;
        os << "Rotation (cardan angles in deg) :\n";
        os << "Cardan angle (deg) : ";
        os << "phi   = " << phi;
        os << "; theta = " << theta;
        os << "; psi   = " << psi;
        os << std::endl;

    }

    std::ostream& operator<<(std::ostream& os, const FrRotation_& rotation) {
        return rotation.cout(os);
    }

    FRAME_CONVENTION FrRotation_::GetFrameConvention() const {
        return m_frQuaternion.GetFrameConvention();
    }

    FRAME_CONVENTION FrRotation_::SwapAbsFrameConvention() {
        return m_frQuaternion.SwapAbsFrameConvention();
    }

    void FrRotation_::SetFrameConvention(FRAME_CONVENTION fc) {
        m_frQuaternion.SetFrameConvention(fc);
    }

    void FrRotation_::SetNWU() {
        m_frQuaternion.SetNWU();
    }

    void FrRotation_::SetNED() {
        m_frQuaternion.SetNED();
    }






}  // end namespace frydom