//
// Created by frongere on 20/09/18.
//

#include "FrRotation.h"

#include "FrMatrix.h"

#include "FrEulerAngles.h"

#include "FrGeographic.h"


namespace frydom {


    // FrUnitQuaternion_

    FrUnitQuaternion_::FrUnitQuaternion_() : m_chronoQuaternion() {
        Normalize();
    } // OK

    FrUnitQuaternion_::FrUnitQuaternion_(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc) { // OK
        Set(q0, q1, q2, q3, fc);
    }

    FrUnitQuaternion_::FrUnitQuaternion_(double q0, double q1, double q2, double q3, bool non_normalized, FRAME_CONVENTION fc) { // OK
        Set(q0, q1, q2, q3, non_normalized, fc);
    }

    FrUnitQuaternion_::FrUnitQuaternion_(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) {  // OK
        Set(axis, angleRAD, fc);
    }

    FrUnitQuaternion_::FrUnitQuaternion_(const FrUnitQuaternion_ &other) {
        Set(other);
    }

    void FrUnitQuaternion_::Set(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc) {  // OK
        Set(q0, q1, q2, q3, false, fc);
    }

    void FrUnitQuaternion_::Set(double q0, double q1, double q2, double q3, bool non_normalized, FRAME_CONVENTION fc) {  // OK
        if (IsNED(fc)) internal::SwapQuaternionElementsFrameConvention(q0, q1, q2, q3); // Internal chrono quaternion must always be in NWU convention
        m_chronoQuaternion.Set(q0, q1, q2, q3);
        if (non_normalized) Normalize();
        else assert(IsRotation());

    }

    void FrUnitQuaternion_::Set(const FrUnitQuaternion_ &quaternion) {  // OK
        m_chronoQuaternion.Set(quaternion.m_chronoQuaternion);
        assert(IsRotation());
    }

    void FrUnitQuaternion_::Set(const Direction& axis, double angleRAD, FRAME_CONVENTION fc) {  // OK
        assert((axis.norm()-1.) < 1e-8);
        auto axisTmp = axis;
        if (IsNED(fc)) internal::SwapFrameConvention<Direction>(axisTmp);
        m_chronoQuaternion = chrono::Q_from_AngAxis(angleRAD, internal::Vector3dToChVector(axisTmp));
        assert(IsRotation());
    }

    void FrUnitQuaternion_::SetNullRotation() {  // OK
        m_chronoQuaternion.SetUnit();
    }

    void FrUnitQuaternion_::Normalize() {  // OK
        m_chronoQuaternion.Normalize();
    }

    double FrUnitQuaternion_::Norm() const {  // OK
        return m_chronoQuaternion.Length();
    }

    bool FrUnitQuaternion_::IsRotation() const {  // OK
        return (mathutils::IsClose<double>(Norm(), 1.));
    }

    void FrUnitQuaternion_::Get(double &q0, double &q1, double &q2, double &q3, FRAME_CONVENTION fc) const {  // OK
        q0 = m_chronoQuaternion[0];
        q1 = m_chronoQuaternion[1];
        q2 = m_chronoQuaternion[2];
        q3 = m_chronoQuaternion[3];  // In NWU
        if (IsNED(fc)) internal::SwapQuaternionElementsFrameConvention(q0, q1, q2, q3); // Internal chrono quaternion is always in NWU convention
    }

    void FrUnitQuaternion_::Get(Direction& axis, double& angleRAD, FRAME_CONVENTION fc) const {  // OK
        chrono::ChVector<double> chronoAxis;
        chrono::Q_to_AngAxis(m_chronoQuaternion, angleRAD, chronoAxis); // In NWU
        axis = internal::ChVectorToVector3d<Direction>(chronoAxis); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Direction>(axis);
    }

    Direction FrUnitQuaternion_::GetXAxis(FRAME_CONVENTION fc) const {  // OK
        auto axis = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetXaxis());  // NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Direction>(axis);
        return axis;
    }

    Direction FrUnitQuaternion_::GetYAxis(FRAME_CONVENTION fc) const {  // OK
        auto axis = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetYaxis());  // NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Direction>(axis);
        return axis;
    }

    Direction FrUnitQuaternion_::GetZAxis(FRAME_CONVENTION fc) const {  // OK
        auto axis = internal::ChVectorToVector3d<Direction>(m_chronoQuaternion.GetZaxis());  // NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Direction>(axis);
        return axis;
    }

    FrUnitQuaternion_ &FrUnitQuaternion_::operator=(const FrUnitQuaternion_ &other) {  // OK
        m_chronoQuaternion = other.m_chronoQuaternion;
    }

    FrUnitQuaternion_ FrUnitQuaternion_::operator*(const FrUnitQuaternion_ &other) const {  // OK
        auto newQuat = FrUnitQuaternion_(*this);
        newQuat *= other;
        return newQuat;
    }

    FrUnitQuaternion_ &FrUnitQuaternion_::operator*=(const FrUnitQuaternion_ &other) {  // OK
//        m_chronoQuaternion *= other.m_chronoQuaternion; // FIXME : this Chrono operator is currently buggy (07/11/2018)
        m_chronoQuaternion = m_chronoQuaternion * other.m_chronoQuaternion;
        // TODO : wait for a new release of Chrono for a fix to *= to use it... A message has been sent to the Chrono list by Lucas (07/11/2018)
        return *this;
    }

    bool FrUnitQuaternion_::operator==(const frydom::FrUnitQuaternion_ &other) const {
        return m_chronoQuaternion == other.m_chronoQuaternion;
    }

    const chrono::ChQuaternion<double>& FrUnitQuaternion_::GetChronoQuaternion() const {  // TODO : supprimer le besoin de cette methode
        return m_chronoQuaternion;
    }

    FrUnitQuaternion_& FrUnitQuaternion_::Inverse() {  // OK
        // We can use only m_chronoQuaternion.Conjugate() and not m_chronoQuaternion.Inverse()
        // because we know for sure our Unit quaternion is already normalized.
        m_chronoQuaternion.Conjugate();
        return *this;
    }

    FrUnitQuaternion_ FrUnitQuaternion_::GetInverse() const {  // OK
        return FrUnitQuaternion_(*this).Inverse();
    }

    std::ostream& FrUnitQuaternion_::cout(std::ostream &os) const {  // OK

        os << std::endl;
        os << "Quaternion : q0 = "<< m_chronoQuaternion.e0()
           <<", q1 = "<< m_chronoQuaternion.e1()
           <<", q2 = "<< m_chronoQuaternion.e2()
           <<", q3 = "<< m_chronoQuaternion.e3()
           << std::endl;

        return os;
    }

    std::ostream& operator<<(std::ostream& os, const FrUnitQuaternion_& quaternion) {  // OK
        return quaternion.cout(os);
    }

    mathutils::Matrix33<double> FrUnitQuaternion_::GetRotationMatrix() const {
        chrono::ChMatrix33<double> chronoMatrix;
        chronoMatrix.Set_A_quaternion(m_chronoQuaternion);
        return internal::ChMatrix33ToMatrix33(chronoMatrix);
    }

    mathutils::Matrix33<double> FrUnitQuaternion_::GetInverseRotationMatrix() const {
        chrono::ChMatrix33<double> chronoMatrix;
        chronoMatrix.Set_A_quaternion(m_chronoQuaternion.GetInverse());
        return internal::ChMatrix33ToMatrix33(chronoMatrix);
    }

    mathutils::Matrix33<double> FrUnitQuaternion_::LeftMultiply(const mathutils::Matrix33<double>& matrix) const {
        return GetRotationMatrix() * matrix;
    }

    mathutils::Matrix33<double> FrUnitQuaternion_::RightMultiply(const mathutils::Matrix33<double>& matrix) const {
        return matrix * GetRotationMatrix();
    }

    mathutils::Matrix33<double> FrUnitQuaternion_::LeftMultiplyInverse(const mathutils::Matrix33<double>& matrix) const {
        return GetInverseRotationMatrix() * matrix;
    }

    mathutils::Matrix33<double> FrUnitQuaternion_::RightMultiplyInverse(const mathutils::Matrix33<double>& matrix) const {
        return matrix * GetInverseRotationMatrix();
    }

    // FrRotation_

    FrRotation_::FrRotation_() : m_frQuaternion() {}  // OK

    FrRotation_::FrRotation_(FrUnitQuaternion_ quaternion) : m_frQuaternion(quaternion) {}  // OK

    FrRotation_::FrRotation_(const Direction& axis, double angleRAD, FRAME_CONVENTION fc) :
                        m_frQuaternion(axis, angleRAD, fc) {}  // OK

    void FrRotation_::SetNullRotation() {  // OK
        m_frQuaternion.SetNullRotation();
    }

    void FrRotation_::Set(const FrUnitQuaternion_& quat) {  // OK
        m_frQuaternion.Set(quat);
    }

    FrUnitQuaternion_& FrRotation_::GetQuaternion() {  // OK
        return m_frQuaternion;
    }

    const FrUnitQuaternion_& FrRotation_::GetQuaternion() const {  // OK
        return m_frQuaternion;
    }

    void FrRotation_::SetAxisAngle(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) {  // OK
        m_frQuaternion.Set(axis, angleRAD, fc);
    }

    void FrRotation_::GetAxisAngle(Direction &axis, double angleRAD, FRAME_CONVENTION fc) {  // OK
        m_frQuaternion.Get(axis, angleRAD, fc);
    }

    void FrRotation_::GetAxis(Direction &axis, FRAME_CONVENTION fc) {  // OK
        double angle=0.;
        GetAxisAngle(axis, angle, fc);
    }

    void FrRotation_::GetAngle(double &angle) {  // OK
        Direction axis;
        GetAxisAngle(axis, angle, NWU);
    }

    mathutils::Matrix33<double> FrRotation_::GetRotationMatrix() const {
        return m_frQuaternion.GetRotationMatrix();
    }

    mathutils::Matrix33<double> FrRotation_::GetInverseRotationMatrix() const{
        return m_frQuaternion.GetInverseRotationMatrix();
    }

    void FrRotation_::SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {  // OK

        if (IsNED(fc)) {  // FIXME : la convention n'est valable que pour une sequence se terminant par yz...
            // Conver it to NWU convention
            theta = -theta;
            psi   = -psi;
        }

        m_frQuaternion.Set(internal::Ch2FrQuaternion(internal::euler_to_quat(phi, theta, psi, seq)));

    }

    void FrRotation_::SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {  // OK
        SetEulerAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, seq, fc);
    }

    void FrRotation_::SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc) {  // OK
        SetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
    }

    void FrRotation_::SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc) {  // OK
        SetCardanAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, fc);
    }

    void FrRotation_::GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {  // OK
        // FIXME : voir a supprimer ce besoin de friend method !!!
        // Par exemple en ajoutant des fonctions ne prenant que des coefficients de quaternions dans EulerAngles.h
        auto vec = internal::quat_to_euler(m_frQuaternion.GetChronoQuaternion(), seq);  // In NWU  // TODO : avoir methode ne manipulat pas de ChVector
        phi   = vec[0];
        theta = vec[1];
        psi   = vec[2];

        if (IsNED(fc)) {  // FIXME : la convention n'est valable que pour une sequence se terminant par yz...
            // Convert it to NED convention
            theta = -theta;
            psi   = -psi;
        }
    }

    void FrRotation_::GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const {  // OK
        GetEulerAngles_RADIANS(phi, theta, psi, seq, fc);
        phi   *= RAD2DEG;
        theta *= RAD2DEG;
        psi   *= RAD2DEG;
    }

    void FrRotation_::GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {  // OK
        GetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
    }

    void FrRotation_::GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {  // OK
        GetEulerAngles_DEGREES(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
    }

    void FrRotation_::GetFixedAxisAngles_RADIANS(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const {
        // TODO : utiliser
    }

    void FrRotation_::GetFixedAxisAngles_DEGREES(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const {
        // TODO
    }

    FrRotation_& FrRotation_::operator=(const FrRotation_ &other) {  // OK
        m_frQuaternion = other.m_frQuaternion;
    }

    FrRotation_ FrRotation_::operator*(const FrRotation_ &other) const {  // OK
        return FrRotation_(m_frQuaternion * other.m_frQuaternion);
    }

    FrRotation_ & FrRotation_::operator*=(const FrRotation_ &other) {  // OK
        m_frQuaternion *= other.m_frQuaternion;
    }

    bool FrRotation_::operator==(const FrRotation_& other) const {
        return m_frQuaternion == other.m_frQuaternion;
    }

    mathutils::Matrix33<double> FrRotation_::LeftMultiplyInverse(const mathutils::Matrix33<double>& matrix) const {
        return GetInverseRotationMatrix() * matrix;
    }

    mathutils::Matrix33<double> FrRotation_::RighttMultiply(const mathutils::Matrix33<double>& matrix) const {
        return matrix * GetRotationMatrix();
    }

    mathutils::Matrix33<double> FrRotation_::RighttMultiplyInverse(const mathutils::Matrix33<double>& matrix) const {
        return matrix * GetInverseRotationMatrix();
    }


//    Position FrRotation_::Rotate(const Position &vector, FRAME_CONVENTION fc) {  // OK
//        auto out = m_frQuaternion.Rotate(vector, fc);
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(out);
//        return out;
//    }

    FrRotation_& FrRotation_::RotAxisAngle_RADIANS(const Direction &axis, double angle, FRAME_CONVENTION fc) {  // OK
        *this *= FrRotation_(axis, angle, fc);
        return *this;
    }

    FrRotation_& FrRotation_::RotAxisAngle_DEGREES(const Direction &axis, double angle, FRAME_CONVENTION fc) {  // OK
        RotAxisAngle_RADIANS(axis, angle*DEG2RAD, fc);
    }

    FrRotation_& FrRotation_::RotX_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        RotAxisAngle_RADIANS(Direction(1., 0., 0.), angle, fc);
    }

    FrRotation_& FrRotation_::RotX_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        RotX_RADIANS(angle*DEG2RAD, fc);
    }

    FrRotation_& FrRotation_::RotY_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        RotAxisAngle_RADIANS(Direction(0., 1., 0.), angle, fc);
    }

    FrRotation_& FrRotation_::RotY_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        RotY_RADIANS(angle*DEG2RAD, fc);
    }

    FrRotation_& FrRotation_::RotZ_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        RotAxisAngle_RADIANS(Direction(0., 0., 1.), angle, fc);
    }

    FrRotation_& FrRotation_::RotZ_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        RotZ_RADIANS(angle*DEG2RAD, fc);
    }

    Direction FrRotation_::GetXAxis(FRAME_CONVENTION fc) const {  // OK
        return m_frQuaternion.GetXAxis(fc);
    }

    Direction FrRotation_::GetYAxis(FRAME_CONVENTION fc) const {  // OK
        return m_frQuaternion.GetYAxis(fc);
    }

    Direction FrRotation_::GetZAxis(FRAME_CONVENTION fc) const {  // OK
        return m_frQuaternion.GetZAxis(fc);
    }

    std::ostream& FrRotation_::cout(std::ostream &os) const {  // OK

        double phi, theta, psi;
        GetCardanAngles_DEGREES(phi, theta, psi, NWU);

        os << std::endl;
        os << "Rotation (cardan angles in deg, NWU convention) :\n";
        os << "Cardan angle (deg) : ";
        os << "phi   = " << phi;
        os << "; theta = " << theta;
        os << "; psi   = " << psi;
        os << std::endl;

    }

    std::ostream& operator<<(std::ostream& os, const FrRotation_& rotation) {  // OK
        return rotation.cout(os);
    }

}  // end namespace frydom
