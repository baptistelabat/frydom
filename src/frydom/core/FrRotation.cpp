//
// Created by frongere on 20/09/18.
//

#include "FrRotation.h"

#include "FrEulerAngles.h"

#include "MathUtils/Vector3d.h"


namespace frydom {

    namespace internal {


        // Conversion functions
        Vector3d ChVector2Vector3d(const chrono::ChVector<double> vector) {
            Vector3d vector3d;
            vector3d << vector.x(), vector.y(), vector.z();
            return vector3d;
        }

        chrono::ChVector<double> Vector3d2ChVector(const Vector3d vector3d) {
            chrono::ChVector<double> vector(vector3d[0], vector3d[1], vector3d[2]);
            return vector;
        }

        chrono::ChQuaternion<double> FrQuaternion2ChQuaternion(const FrQuaternion_ &frQuaternion) {
            return chrono::ChQuaternion<double>(frQuaternion.GetScalar(), Vector3d2ChVector(frQuaternion.GetVector()));
        }

        FrQuaternion_ ChQuaternion2FrQuaternion(const chrono::ChQuaternion<double> &chQuaternion) {
            return FrQuaternion_();
        }


    }


    FrQuaternion_::FrQuaternion_() : m_chronoQuaternion() {}

    FrQuaternion_::FrQuaternion_(double q0, double q1, double q2, double q3) : m_chronoQuaternion(q0, q1, q2, q3) {}

    FrQuaternion_::FrQuaternion_(double s, const Vector3d imag) : m_chronoQuaternion(s, internal::Vector3d2ChVector(imag)) {}

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
        m_chronoQuaternion.SetVector(internal::Vector3d2ChVector(v));
    }

    double FrQuaternion_::GetScalar() const {
        return m_chronoQuaternion.e0();
    }

    Vector3d FrQuaternion_::GetVector() const {
        return internal::ChVector2Vector3d(m_chronoQuaternion.GetVector());
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
        return internal::ChQuaternion2FrQuaternion(-m_chronoQuaternion);
    }

    FrQuaternion_ FrQuaternion_::operator*(const FrQuaternion_ &other) const {
        return internal::ChQuaternion2FrQuaternion(m_chronoQuaternion * other.m_chronoQuaternion);
    }

    FrQuaternion_ &FrQuaternion_::operator*=(const FrQuaternion_ &other) {
        m_chronoQuaternion *= other.m_chronoQuaternion;
    }

    Vector3d FrQuaternion_::Rotate(const Vector3d &vector) {
        return internal::ChVector2Vector3d(m_chronoQuaternion.Rotate(internal::Vector3d2ChVector(vector)));
    }

    chrono::ChQuaternion<double> FrQuaternion_::GetChronoQuaternion() const {
        return m_chronoQuaternion;
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
        m_quaternion = internal::ChQuaternion2FrQuaternion(internal::axis_angle_to_quat(internal::Vector3d2ChVector(axis), angle));
    }

    void FrRotation_::GetAxisAngle(Vector3d &axis, double angle) {
        chrono::ChVector<double> vec;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
        axis = internal::ChVector2Vector3d(vec);
    }

    void FrRotation_::GetAxis(Vector3d &axis) {
        chrono::ChVector<double> vec;
        double angle;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
        axis = internal::ChVector2Vector3d(vec);
    }

    void FrRotation_::GetAngle(double &angle) {
        chrono::ChVector<double> vec;
        internal::quat_to_axis_angle(m_quaternion.GetChronoQuaternion(), vec, angle);
    }

    void FrRotation_::SetRotationMatrix(const FrRotationMatrix_ &mat) {
        m_quaternion = internal::ChQuaternion2FrQuaternion(internal::mat_to_quat(mat.GetChMatrix()));
    }

    FrRotationMatrix_ FrRotation_::GetRotationMatrix() const {
        FrRotationMatrix_ matrix;
        matrix.m_matrix = internal::quat_to_mat(m_quaternion.GetChronoQuaternion());
        return matrix;
    }

    void FrRotation_::SetEulerAngles(double phi, double theta, double psi, EulerSeq seq) {
        m_quaternion = internal::ChQuaternion2FrQuaternion(internal::euler_to_quat(phi, theta, psi, seq));
    }

    void FrRotation_::SetCardanAngles(double phi, double theta, double psi) {
        SetEulerAngles(phi, theta, psi, EulerSeq::CARDAN);
    }

    void FrRotation_::GetEulerAngles(double &phi, double &theta, double &psi, EulerSeq seq) const {
        auto vec = internal::quat_to_euler(m_quaternion.GetChronoQuaternion(), seq);
        phi = vec[0];
        theta = vec[1];
        psi = vec[2];
    }

    void FrRotation_::GetCardanAngles(double &phi, double &theta, double &psi) const {
        GetEulerAngles(phi, theta, psi, EulerSeq::CARDAN);
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