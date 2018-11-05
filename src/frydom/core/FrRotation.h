//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRROTATION_H
#define FRYDOM_FRROTATION_H


#include "chrono/core/ChQuaternion.h"
#include "FrRotation.h"
#include "FrGeographic.h"

#include "FrEulerAngles.h"

#include "FrVector.h"


namespace frydom {


    // WARNING: chrono object must always be in NWU convention. Conversion to NED must be applied on FRyDoM objects


    class FrQuaternion_ {

    private:

        chrono::ChQuaternion<double> m_chronoQuaternion;  // Chrono objects are always stored in NWU frame convention

        const chrono::ChQuaternion<double>& GetChronoQuaternion() const;

        friend class FrRotation_;


    public:

        FrQuaternion_();

        FrQuaternion_(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc);

        FrQuaternion_(const Direction &axis, double angleRAD, FRAME_CONVENTION fc);

        FrQuaternion_(const FrQuaternion_& other);

        void Set(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc);

        void Set(const FrQuaternion_& quaternion);

        void Set(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        void SetNull();

        void Normalize();

        double Norm() const;

        bool IsRotation() const;

        void Get(double& q0, double& q1, double& q2, double& q3, FRAME_CONVENTION fc) const;

        void Get(Direction& axis, double& angleRAD, FRAME_CONVENTION fc) const;

        Direction GetXAxis(FRAME_CONVENTION fc) const;

        Direction GetYAxis(FRAME_CONVENTION fc) const;

        Direction GetZAxis(FRAME_CONVENTION fc) const;


        // Operators

        FrQuaternion_& operator=(const FrQuaternion_& other);

        FrQuaternion_ operator*(const FrQuaternion_& other) const;

        FrQuaternion_& operator*=(const FrQuaternion_& other);

        // TODO : voir pour rendre generique par rapport aux differents vecteurs...
        template <class Vector>
        Vector Rotate(const Vector& vector, FRAME_CONVENTION fc) {
            auto vectorTmp = vector;

            if (IsNED(fc)) internal::SwapFrameConvention<Vector>(vectorTmp);

            auto chronoVector = internal::Vector3dToChVector(vectorTmp);

            vectorTmp = internal::ChVectorToVector3d<Vector>(m_chronoQuaternion.Rotate(chronoVector));

            if (IsNED(fc)) internal::SwapFrameConvention<Vector>(vectorTmp);

            return vectorTmp;
        }

        FrQuaternion_& Inverse();

        FrQuaternion_ GetInverse() const;

    };



    class FrRotation_ {

    private:

        FrQuaternion_ m_frQuaternion;  ///< The internal quaternion.

    public:

        /// Default rotation constructor. This is the null rotation
        FrRotation_();

        /// Constructor from a quaternion
        explicit FrRotation_(FrQuaternion_ quaternion);

        /// Constructor from axis angle rotation (angle in radians)
        FrRotation_(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        /// Set the null rotation
        void SetNull();


        // Quaternion representation

        /// Set the quaternion of the rotation
        void Set(const FrQuaternion_ &quat);

        /// Get the quaternion
        FrQuaternion_& GetQuaternion();

        /// Get the quaternion (const)
        const FrQuaternion_& GetQuaternion() const;


        // Axis angle representation

        /// Set the rotation by axis angle representation (angle in radians)
        void SetAxisAngle(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        /// Get the rotation by axis angle representation (angle in radians)
        void GetAxisAngle(Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        /// Get the axis of the rotation
        void GetAxis(Direction& axis, FRAME_CONVENTION fc);

        /// Get the angle in rotation in space (in radians)
        void GetAngle(double& angle);


        // Euler angles representation

        /// Set the euler angles following the prescribed axis sequence (angles in radians)
        void SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        /// Set the euler angles following the prescribed axis sequence (angles in degrees)
        void SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        /// Set the cardan angles (angles in radians)
        void SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc);

        /// Set the cardan angles (angles in degrees)
        void SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc);

        /// Get the euler angles following the prescribed axis sequence (angles in radians)
        void GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        /// Get the euler angles following the prescribed axis sequence (angles in degrees)
        void GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        /// Get the cardan angles (angles in radians)
        void GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const;

        /// Get the cardan angles (angles in degrees)
        void GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const;

        /// Get the angles of a fixed axis representation (angles in radians)
        void GetFixedAxisAngles_RADIANS(double& rx, double& ry, double& rz, FRAME_CONVENTION fc) const;

        /// Get the angles of a fixed axis representation (angles in degrees)
        void GetFixedAxisAngles_DEGREES(double& rx, double& ry, double& rz, FRAME_CONVENTION fc) const;


        // Helpers to build rotations

        /// Apply a rotation to the current rotation by the axis angle representation (angle in radians)
        FrRotation_& RotAxisAngle_RADIANS(const Direction& axis, double angle, FRAME_CONVENTION fc);

        /// Apply a rotation to the current rotation by the axis angle representation (angle in degrees)
        FrRotation_& RotAxisAngle_DEGREES(const Direction& axis, double angle, FRAME_CONVENTION fc);

        /// Apply a rotation around the X axis to the current rotation (angle in radians)
        FrRotation_& RotX_RADIANS(double angle, FRAME_CONVENTION fc);

        /// Apply a rotation around the X axis to the current rotation (angle in degrees)
        FrRotation_& RotX_DEGREES(double angle, FRAME_CONVENTION fc);

        /// Apply a rotation around the Y axis to the current rotation (angle in radians)
        FrRotation_& RotY_RADIANS(double angle, FRAME_CONVENTION fc);

        /// Apply a rotation around the Y axis to the current rotation (angle in degrees)
        FrRotation_& RotY_DEGREES(double angle, FRAME_CONVENTION fc);

        /// Apply a rotation around the Z axis to the current rotation (angle in radians)
        FrRotation_& RotZ_RADIANS(double angle, FRAME_CONVENTION fc);

        /// Apply a rotation around the Z axis to the current rotation (angle in degrees)
        FrRotation_& RotZ_DEGREES(double angle, FRAME_CONVENTION fc);


        // Operators

        /// Assign the other rotation to the current rotation
        FrRotation_& operator=(const FrRotation_& other);

        /// Compose the current rotation with the other. In a matrix form it would be result = this*other.
        FrRotation_ operator*(const FrRotation_& other) const;

        /// Compopse the current rotation with the other inplace. In a matrix form it would be this = this*other.
        FrRotation_&operator*=(const FrRotation_& other);

        /// Rotate a vector by the current rotation. Templatized method by the type of vector (Position, Velocity... cf FrVector.h)
        template <class Vector>
        Vector Rotate(const Vector& vector, FRAME_CONVENTION fc) {
            auto out = m_frQuaternion.Rotate<Vector>(vector, fc);
            if (IsNED(fc)) internal::SwapFrameConvention<Vector>(out);
            return out;
        }

        /// Get the X axis of the coordinate system obtained by the rotation
        Direction GetXAxis(FRAME_CONVENTION fc) const;

        /// Get the Y axis of the coordinate system obtained by the rotation
        Direction GetYAxis(FRAME_CONVENTION fc) const;

        /// Get the Z axis of the coordinate system obtained by the rotation
        Direction GetZAxis(FRAME_CONVENTION fc) const;


        friend std::ostream& operator<<(std::ostream& os, const FrRotation_& rotation);


    private:

        std::ostream& cout(std::ostream& os) const;

    };


    namespace internal {

        // Conversion functions between Chrono quaternions and FRyDoM quaternions

        /// Convert a FRyDoM quaternion into a Chrono quaternion
        inline chrono::ChQuaternion<double> Fr2ChQuaternion(const FrQuaternion_& frQuaternion) {  // OK
            double q0, q1, q2, q3;
            frQuaternion.Get(q0, q1, q2, q3, NWU);
            return chrono::ChQuaternion<double>(q0, q1, q2, q3);
        }

        /// Convert a Chrono quaternion into a FRyDoM quaternion
        inline FrQuaternion_ Ch2FrQuaternion(const chrono::ChQuaternion<double>& chQuaternion) {  // OK
            return FrQuaternion_(chQuaternion.e0(), chQuaternion.e1(), chQuaternion.e2(), chQuaternion.e3(), NWU);
        }

        /// Swap the frame convention (NED/NWU) of quaternion coefficients
        inline void SwapQuaternionElementsFrameConvention(double& q0, double& q1, double& q2, double& q3) {
            q2 = -q2;
            q3 = -q3;
        }

    }

}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
