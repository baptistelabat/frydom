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

        bool operator==(const FrQuaternion_& other) const;

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

        friend std::ostream& operator<<(std::ostream& os, const FrQuaternion_& quaternion);


    private:

        std::ostream& cout(std::ostream& os) const;

    };



    class FrRotation_ {

    private:

        FrQuaternion_ m_frQuaternion;

    public:

        /// Default rotation constructor. This is the null rotation
        FrRotation_();

        explicit FrRotation_(FrQuaternion_ quaternion);

        FrRotation_(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        void SetNull();


        // Quaternion representation

        void Set(const FrQuaternion_ &quat);

        FrQuaternion_& GetQuaternion();

        const FrQuaternion_& GetQuaternion() const;


        // Axis angle representation

        void SetAxisAngle(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        void GetAxisAngle(Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        void GetAxis(Direction& axis, FRAME_CONVENTION fc);

        void GetAngle(double& angle);


        // Euler angles representation

        void SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);
        
        void SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        void SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc);

        void SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc);

        void GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        void GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        void GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const;

        void GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const;

        void GetFixedAxisAngles_RADIANS(double& rx, double& ry, double& rz, FRAME_CONVENTION fc) const;

        void GetFixedAxisAngles_DEGREES(double& rx, double& ry, double& rz, FRAME_CONVENTION fc) const;


        // Helpers to build rotations

        FrRotation_& RotAxisAngle_RADIANS(const Direction& axis, double angle, FRAME_CONVENTION fc);

        FrRotation_& RotAxisAngle_DEGREES(const Direction& axis, double angle, FRAME_CONVENTION fc);

        FrRotation_& RotX_RADIANS(double angle, FRAME_CONVENTION fc);

        FrRotation_& RotX_DEGREES(double angle, FRAME_CONVENTION fc);

        FrRotation_& RotY_RADIANS(double angle, FRAME_CONVENTION fc);

        FrRotation_& RotY_DEGREES(double angle, FRAME_CONVENTION fc);

        FrRotation_& RotZ_RADIANS(double angle, FRAME_CONVENTION fc);

        FrRotation_& RotZ_DEGREES(double angle, FRAME_CONVENTION fc);


        // Operators

        FrRotation_& operator=(const FrRotation_& other);

        FrRotation_ operator*(const FrRotation_& other) const;

        FrRotation_&operator*=(const FrRotation_& other);

        bool operator==(const FrRotation_& other) const;

        template <class Vector>
        Vector Rotate(const Vector& vector, FRAME_CONVENTION fc) {
            auto out = m_frQuaternion.Rotate<Vector>(vector, fc);
            if (IsNED(fc)) internal::SwapFrameConvention<Vector>(out);
            return out;
        }

        Direction GetXAxis(FRAME_CONVENTION fc) const;

        Direction GetYAxis(FRAME_CONVENTION fc) const;

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

        inline void SwapQuaternionElementsFrameConvention(double& q0, double& q1, double& q2, double& q3) {
            q2 = -q2;
            q3 = -q3;
        }

    }

}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
