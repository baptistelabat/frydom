//
// Created by frongere on 20/09/18.
//

#include "FrRotation.h"

#include "FrEulerAngles.h"

#include "FrGeographic.h"


namespace frydom {

    namespace internal {

        // Conversion functions



    }


    // FrQuaternion_

    FrQuaternion_::FrQuaternion_() : m_chronoQuaternion() {}

    FrQuaternion_::FrQuaternion_(double q0, double q1, double q2, double q3) : m_chronoQuaternion(q0, q1, q2, q3) {}

    FrQuaternion_::FrQuaternion_(double s, const Direction imag) : m_chronoQuaternion(s, internal::Vector3dToChVector(imag)) {}

    FrQuaternion_::FrQuaternion_(const FrQuaternion_ &other) = default;

    void FrQuaternion_::Set(double q0, double q1, double q2, double q3) {
        m_chronoQuaternion.Set(q0, q1, q2, q3);
    }

    void FrQuaternion_::Set(const FrQuaternion_ &quaternion) {
        m_chronoQuaternion.Set(quaternion.m_chronoQuaternion);
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

    void FrQuaternion_::SetScalar(double s) {
        m_chronoQuaternion.SetScalar(s);
    }

    void FrQuaternion_::SetDirection(const Direction &v) {
        m_chronoQuaternion.SetVector(internal::Vector3dToChVector(v));
    }

    double FrQuaternion_::GetScalar() const {
        return m_chronoQuaternion.e0();
    }

    Direction FrQuaternion_::GetDirection() const {
        return internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetVector(), m_frameConvention);
    }

    void FrQuaternion_::Get(double &q0, double &q1, double &q2, double &q3) const {
        q0 = m_chronoQuaternion[0];
        q1 = m_chronoQuaternion[1];
        q2 = m_chronoQuaternion[2];
        q3 = m_chronoQuaternion[3];
    }

    FRAME_CONVENTION FrQuaternion_::GetFrameConvention() const { return m_frameConvention; }

    FRAME_CONVENTION FrQuaternion_::SwapAbsFrameConvention() {

        internal::swap_NED_NWU(m_chronoQuaternion);

        if (IsNWU(m_frameConvention)) {
            m_frameConvention = NED;
        } else {
            m_frameConvention = NWU;
        }

        return m_frameConvention;
    }

    void FrQuaternion_::SetFrameConvention(FRAME_CONVENTION frameConvention, bool change) {
        if (m_frameConvention != frameConvention) {
            if (change) {
                SwapAbsFrameConvention();
            }
            m_frameConvention = frameConvention;
        }
    }

    void FrQuaternion_::SetNWU() {
        SetFrameConvention(NWU, true);
    }

    void FrQuaternion_::SetNED() {
        SetFrameConvention(NED, true);
    }

//    bool FrQuaternion_::IsAbsolute() const {
//        return IsAbsoluteConvention(m_frameConvention);
//    }
//
//    bool FrQuaternion_::IsRelative() const {
//        return IsRelativeConvention(m_frameConvention);
//    }

    FrQuaternion_ &FrQuaternion_::operator=(const FrQuaternion_ &other) {
        m_chronoQuaternion = other.m_chronoQuaternion;
    }

    FrQuaternion_ FrQuaternion_::operator+() const {
        return *this;
    }

    FrQuaternion_ FrQuaternion_::operator-() const {
        return internal::Ch2FrQuaternion(-m_chronoQuaternion);
    }

    FrQuaternion_ FrQuaternion_::operator*(const FrQuaternion_ &other) const {
        return internal::Ch2FrQuaternion(m_chronoQuaternion * other.m_chronoQuaternion);
    }

    FrQuaternion_ &FrQuaternion_::operator*=(const FrQuaternion_ &other) {
        m_chronoQuaternion *= other.m_chronoQuaternion;
    }

    Position FrQuaternion_::Rotate(const Position &vector) {
        return internal::ChVectorToVector3d<Position>(m_chronoQuaternion.Rotate(internal::Vector3dToChVector(vector)),
                                                      m_frameConvention);
    }

    chrono::ChQuaternion<double> FrQuaternion_::GetChronoQuaternion() const {
        return m_chronoQuaternion;
    }

    FrQuaternion_ &FrQuaternion_::Inverse() {
        m_chronoQuaternion.Conjugate(); // TODO : verifier
    }

    FrQuaternion_ FrQuaternion_::Inverse() const {
        return FrQuaternion_(*this).Inverse();  // TODO verifier
    }


    // FrRotationMatrix

    FrRotationMatrix_::FrRotationMatrix_() {
        m_matrix.SetIdentity();
    }

    FrRotationMatrix_::FrRotationMatrix_(const frydom::FrRotationMatrix_ &other) : m_matrix(other.GetChMatrix()) {}

    chrono::ChMatrix33<double> FrRotationMatrix_::GetChMatrix() const {
        return m_matrix;
    }



    // FrRotation_

    FrRotation_::FrRotation_() : m_quaternion() {}

    FrRotation_::FrRotation_(FrQuaternion_ quaternion) : m_quaternion(quaternion) {}

    void FrRotation_::SetNull() {
        m_quaternion.SetNull();
    }

    void FrRotation_::SetQuaternion(const FrQuaternion_ &quat) {
        m_quaternion = quat;
    }

    FrQuaternion_ &FrRotation_::GetQuaternion() {
        return m_quaternion;
    }

    FrQuaternion_ FrRotation_::GetQuaternion() const {
        return m_quaternion;
    }

    void FrRotation_::SetAxisAngle(const Direction &axis, double angle) {
        m_quaternion = internal::Ch2FrQuaternion(internal::axis_angle_to_quat(internal::Vector3dToChVector(axis), angle));
    }

    void FrRotation_::GetAxisAngle(Direction &axis, double angle) {
        chrono::ChVector<double> vec;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
        axis = internal::ChVectorToVector3d<Direction>(vec, GetFrameConvention());
    }

    void FrRotation_::GetAxis(Direction &axis) {
        chrono::ChVector<double> vec;
        double angle;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
        axis = internal::ChVectorToVector3d<Direction>(vec, GetFrameConvention());
    }

    void FrRotation_::GetAngle(double &angle) {
        chrono::ChVector<double> vec;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
    }

    void FrRotation_::SetRotationMatrix(const FrRotationMatrix_ &mat) {
        m_quaternion = internal::Ch2FrQuaternion(internal::mat_to_quat(mat.GetChMatrix()));
    }

    FrRotationMatrix_ FrRotation_::GetRotationMatrix() const {
        FrRotationMatrix_ matrix;
        matrix.m_matrix = internal::quat_to_mat(m_quaternion.GetChronoQuaternion());
        return matrix;
    }

    void FrRotation_::SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq) {
        m_quaternion = internal::Ch2FrQuaternion(internal::euler_to_quat(phi, theta, psi, seq));
    }

    void FrRotation_::SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq) {
        SetEulerAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, seq);
    }

    void FrRotation_::SetCardanAngles_RADIANS(double phi, double theta, double psi) {
        SetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN);
    }

    void FrRotation_::SetCardanAngles_DEGREES(double phi, double theta, double psi) {
        SetCardanAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD);
    }

    void FrRotation_::GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq) const {
        auto vec = internal::quat_to_euler(m_quaternion.GetChronoQuaternion(), seq);
        phi   = vec[0];
        theta = vec[1];
        psi   = vec[2];
    }

    void FrRotation_::GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq) const {
        GetEulerAngles_RADIANS(phi, theta, psi, seq);
        phi   *= RAD2DEG;
        theta *= RAD2DEG;
        psi   *= RAD2DEG;
    }

    void FrRotation_::GetCardanAngles_RADIANS(double &phi, double &theta, double &psi) const {
        GetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN);
    }

    void FrRotation_::GetCardanAngles_DEGREES(double &phi, double &theta, double &psi) const {
        GetEulerAngles_DEGREES(phi, theta, psi, EULER_SEQUENCE::CARDAN);
    }

    void FrRotation_::GetFixedAxisAngles_RADIANS(double &rx, double &ry, double &rz) const {
        // TODO
    }

    void FrRotation_::GetFixedAxisAngles_DEGREES(double &rx, double &ry, double &rz) const {
        // TODO
    }

    FrRotation_ &FrRotation_::operator=(const FrRotation_ &other) {
        m_quaternion = other.m_quaternion;
    }

    FrRotation_ FrRotation_::operator*(const FrRotation_ &other) const {
        FrRotation_ rotation;
        rotation.SetQuaternion(m_quaternion*other.m_quaternion);
        return rotation;
    }

    FrRotation_ &FrRotation_::operator*=(const FrRotation_ &other) {
        m_quaternion *= other.m_quaternion;
    }

    Position FrRotation_::Rotate(const Position &vector) {
        return m_quaternion.Rotate(vector);
    }

    void FrRotation_::RotAxisAngle_RADIANS(const Direction &axis, double angle) {
        FrRotation_ rotation;
        rotation.SetAxisAngle(axis, angle);
        *this *= rotation;
    }

    void FrRotation_::RotAxisAngle_DEGREES(const Direction &axis, double angle) {
        RotAxisAngle_RADIANS(axis, angle*DEG2RAD);
    }

    void FrRotation_::RotX_RADIANS(double angle) {
        RotAxisAngle_RADIANS(Direction(1., 0., 0., GetFrameConvention()), angle);
    }

    void FrRotation_::RotX_DEGREES(double angle) {
        RotX_RADIANS(angle*DEG2RAD);
    }

    void FrRotation_::RotY_RADIANS(double angle) {
        RotAxisAngle_RADIANS(Direction(0., 1., 0., GetFrameConvention()), angle);
    }

    void FrRotation_::RotY_DEGREES(double angle) {
        RotY_RADIANS(angle*DEG2RAD);
    }

    void FrRotation_::RotZ_RADIANS(double angle) {
        RotAxisAngle_RADIANS(Direction(0., 0., 1., GetFrameConvention()), angle);
    }

    void FrRotation_::RotZ_DEGREES(double angle) {
        RotZ_RADIANS(angle*DEG2RAD);
    }

    Direction FrRotation_::GetXAxis() const {
        return internal::ChVectorToVector3d<Direction>(m_quaternion.GetChronoQuaternion().GetXaxis(), GetFrameConvention());
    }

    Direction FrRotation_::GetYAxis() const {
        return internal::ChVectorToVector3d<Direction>(m_quaternion.GetChronoQuaternion().GetYaxis(), GetFrameConvention());
    }

    Direction FrRotation_::GetZAxis() const {
        return internal::ChVectorToVector3d<Direction>(m_quaternion.GetChronoQuaternion().GetZaxis(), GetFrameConvention());
    }




    std::ostream& FrRotation_::cout(std::ostream &os) const {

        double phi, theta, psi;
        GetCardanAngles_DEGREES(phi, theta, psi);

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
        return m_quaternion.GetFrameConvention();
    }

    FRAME_CONVENTION FrRotation_::SwapAbsFrameConvention() {
        return m_quaternion.SwapAbsFrameConvention();
    }

    void FrRotation_::SetFrameConvention(FRAME_CONVENTION frameConvention, bool change) {
        m_quaternion.SetFrameConvention(frameConvention, change);
    }

    void FrRotation_::SetNWU() {
        m_quaternion.SetNWU();
    }

    void FrRotation_::SetNED() {
        m_quaternion.SetNED();
    }






}  // end namespace frydom