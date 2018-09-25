//
// Created by frongere on 20/09/18.
//

#include "FrRotation.h"

#include "FrEulerAngles.h"

#include "MathUtils/Vector3d.h"


namespace frydom {

    namespace internal {

        // Conversion functions
        Vector3d ChVectorToVector3d(const chrono::ChVector<double> vector) {
            Vector3d vector3d;
            vector3d << vector.x(), vector.y(), vector.z();
            return vector3d;
        }

        chrono::ChVector<double> Vector3dToChVector(const Vector3d vector3d) {
            chrono::ChVector<double> vector(vector3d[0], vector3d[1], vector3d[2]);
            return vector;
        }

        chrono::ChQuaternion<double> Fr2ChQuaternion(const FrQuaternion_ &frQuaternion) {
            // TODO : voir si pas conversion plus directe...
            return chrono::ChQuaternion<double>(frQuaternion.GetScalar(), Vector3dToChVector(frQuaternion.GetVector()));
        }

        FrQuaternion_ Ch2FrQuaternion(const chrono::ChQuaternion<double> &chQuaternion) {
            return FrQuaternion_();
        }

    }


    // FrQuaternion_

    FrQuaternion_::FrQuaternion_() : m_chronoQuaternion() {}

    FrQuaternion_::FrQuaternion_(double q0, double q1, double q2, double q3) : m_chronoQuaternion(q0, q1, q2, q3) {}

    FrQuaternion_::FrQuaternion_(double s, const Vector3d imag) : m_chronoQuaternion(s, internal::Vector3dToChVector(imag)) {}

    FrQuaternion_::FrQuaternion_(const FrQuaternion_ &other) : m_chronoQuaternion(other.m_chronoQuaternion) {}

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

    void FrQuaternion_::SetVector(const Vector3d &v) {
        m_chronoQuaternion.SetVector(internal::Vector3dToChVector(v));
    }

    double FrQuaternion_::GetScalar() const {
        return m_chronoQuaternion.e0();
    }

    Vector3d FrQuaternion_::GetVector() const {
        return internal::ChVectorToVector3d(m_chronoQuaternion.GetVector());
    }

    void FrQuaternion_::Get(double &q0, double &q1, double &q2, double &q3) const {
        q0 = m_chronoQuaternion[0];
        q1 = m_chronoQuaternion[1];
        q2 = m_chronoQuaternion[2];
        q3 = m_chronoQuaternion[3];
    }

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

    Vector3d FrQuaternion_::Rotate(const Vector3d &vector) {
        return internal::ChVectorToVector3d(m_chronoQuaternion.Rotate(internal::Vector3dToChVector(vector)));
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

    void FrRotation_::SetAxisAngle(const Vector3d &axis, double angle) {
        m_quaternion = internal::Ch2FrQuaternion(internal::axis_angle_to_quat(internal::Vector3dToChVector(axis), angle));
    }

    void FrRotation_::GetAxisAngle(Vector3d &axis, double angle) {
        chrono::ChVector<double> vec;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
        axis = internal::ChVectorToVector3d(vec);
    }

    void FrRotation_::GetAxis(Vector3d &axis) {
        chrono::ChVector<double> vec;
        double angle;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
        axis = internal::ChVectorToVector3d(vec);
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

    void FrRotation_::SetEulerAngles(double phi, double theta, double psi, EULER_SEQUENCE seq) {
        m_quaternion = internal::Ch2FrQuaternion(internal::euler_to_quat(phi, theta, psi, seq));
    }

    void FrRotation_::SetCardanAngles(double phi, double theta, double psi) {
        SetEulerAngles(phi, theta, psi, EULER_SEQUENCE::CARDAN);
    }

    void FrRotation_::GetEulerAngles(double &phi, double &theta, double &psi, EULER_SEQUENCE seq) const {
        auto vec = internal::quat_to_euler(m_quaternion.GetChronoQuaternion(), seq);
        phi   = vec[0];
        theta = vec[1];
        psi   = vec[2];
    }

    void FrRotation_::GetCardanAngles(double &phi, double &theta, double &psi) const {
        GetEulerAngles(phi, theta, psi, EULER_SEQUENCE::CARDAN);
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

    Vector3d FrRotation_::Rotate(const Vector3d &vector) {
        return m_quaternion.Rotate(vector);
    }



}  // end namespace frydom