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

        FRAME_CONVENTION m_frameConvention = NWU;

        chrono::ChQuaternion<double> m_chronoQuaternion;  // It is always in NWU !!!

        const chrono::ChQuaternion<double>& GetChronoQuaternion() const;

        friend class FrRotation_;


    public:

        explicit FrQuaternion_(FRAME_CONVENTION fc);

        FrQuaternion_(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc);

        FrQuaternion_(const Direction &axis, double angleRAD, FRAME_CONVENTION fc);

        FrQuaternion_(const FrQuaternion_& other);

        void Set(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc);

        void Set(const FrQuaternion_& quaternion);

        void Set(const Direction& axis, double angleRAD);

        void SetNull();

        void Normalize();

        double Norm() const;

        bool IsRotation() const;

//        void SetScalar(double s);
//
//        void SetReal(const Direction &v);

//        double GetScalar() const;

        Direction GetDirection(FRAME_CONVENTION fc) const;

        void Get(double& q0, double& q1, double& q2, double& q3, FRAME_CONVENTION fc) const;

        void Get(Direction& axis, double& angleRAD, FRAME_CONVENTION fc) const;

        Direction GetXAxis(FRAME_CONVENTION fc) const;

        Direction GetYAxis(FRAME_CONVENTION fc) const;

        Direction GetZAxis(FRAME_CONVENTION fc) const;

        FRAME_CONVENTION GetFrameConvention() const;

        FRAME_CONVENTION SwapAbsFrameConvention();

        void SetFrameConvention(FRAME_CONVENTION fc);

        void SetNWU();

        void SetNED();

        bool HasSameConvention(const FrQuaternion_& other) const;

        bool HasSameConvention(FRAME_CONVENTION fc) const;

        template <class Vector>
        bool HasSameConvention(const Vector& vector) const {
            return (m_frameConvention == vector.GetFrameConvention());
        }

//        bool IsAbsolute() const;
//
//        bool IsRelative() const;


        // Operators

        FrQuaternion_& operator=(const FrQuaternion_& other);

//        FrQuaternion_ operator+() const;
//        FrQuaternion_ operator-() const;

        FrQuaternion_ operator*(const FrQuaternion_& other) const;

        FrQuaternion_& operator*=(const FrQuaternion_& other);

        // TODO : voir pour rendre generique par rapport aux differents vecteurs...
        template <class Vector>
        Vector Rotate(const Vector& vector) {
            auto vectorTmp = vector;
            auto fc = vector.GetFrameConvention();

            vectorTmp.SetNWU();

            auto chronoVector = internal::Vector3dToChVector(vectorTmp);

            vectorTmp = internal::ChVectorToVector3d<Vector>(m_chronoQuaternion.Rotate(chronoVector));
            vectorTmp.SetFrameConvention(fc, true);
            return vectorTmp;
        }

        FrQuaternion_& Inverse();

        FrQuaternion_ Inverse() const;

    };


//    // FIXME : on ne devrait pas avoir besoin d'avoir une telle classe, la classe FrRotation_ suffit...
//    class FrRotationMatrix_ {
//
//    private:
//        chrono::ChMatrix33<double> m_matrix;
//
//        chrono::ChMatrix33<double> GetChMatrix() const;
//
//        friend class FrRotation_;
//
//    public:
//        FrRotationMatrix_();
//
//        FrRotationMatrix_(const FrRotationMatrix_& other);
//
//
//
//        // TODO : ajouter des setters et les operations courantes...
//
//
//    };



    class FrRotation_ {

    private:

        FrQuaternion_ m_frQuaternion;

    public:

        /// Default rotation constructor. This is the null rotation
        explicit FrRotation_(FRAME_CONVENTION fc);

        explicit FrRotation_(FrQuaternion_ quaternion);

        FrRotation_(const Direction& axis, double angleRAD);

        void SetNull();


        // Quaternion representation

        void Set(const FrQuaternion_ &quat);

        FrQuaternion_& GetQuaternion();

        FrQuaternion_ GetQuaternion(FRAME_CONVENTION fc) const;


        // Frame conventions

        inline FRAME_CONVENTION GetFrameConvention() const;

        FRAME_CONVENTION SwapAbsFrameConvention();

        void SetFrameConvention(FRAME_CONVENTION fc);

        void SetNWU();

        void SetNED();

        bool HasSameConvention(const FrRotation_& other) const {
            return (GetFrameConvention() == other.GetFrameConvention());
        }

        bool HasSameConvention(FRAME_CONVENTION fc) const {
            return (GetFrameConvention() == fc);
        }

        template <class Vector>
        bool HasSameConvention(const Vector& vector) const {
            return (m_frQuaternion.GetFrameConvention() == vector.GetFrameConvention());
        }


        // Axis angle representation

        void SetAxisAngle(const Direction& axis, double angleRAD);

        void GetAxisAngle(Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        void GetAxis(Direction& axis, FRAME_CONVENTION fc);

        void GetAngle(double& angle);


        // Rotation matrix representation

//        void SetRotationMatrix(const FrRotationMatrix_& mat);

//        FrRotationMatrix_ GetRotationMatrix() const;


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

        FrRotation_& RotAxisAngle_RADIANS(const Direction& axis, double angle);

        FrRotation_& RotAxisAngle_DEGREES(const Direction& axis, double angle);

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

        Position Rotate(const Position& vector);

        Direction GetXAxis(FRAME_CONVENTION fc) const;

        Direction GetYAxis(FRAME_CONVENTION fc) const;

        Direction GetZAxis(FRAME_CONVENTION fc) const;





        friend std::ostream& operator<<(std::ostream& os, const FrRotation_& rotation);

        // TODO : ajouter les GetXAxis etc... cf ChQuaternion pour les methodes...



    private:

        std::ostream& cout(std::ostream& os) const;

    };



    namespace internal {


        inline void swap_NED_NWU(const chrono::ChVector<double> axis, const double angle,
                                 chrono::ChVector<double> &new_axis, double &new_angle) {
            // Angle does not change
            new_angle = angle;

            // Change signs in axis
            new_axis.x() = axis.x();
            new_axis.y() = -axis.y();
            new_axis.z() = -axis.z();  // TODO : a verifier
        }

        inline chrono::ChQuaternion<double>& swap_NED_NWU(chrono::ChQuaternion<double>& quat) {
            quat.e2() = -quat.e2();
            quat.e3() = -quat.e3();
            return quat;
        }

        inline chrono::ChQuaternion<double> swap_NED_NWU(const chrono::ChQuaternion<double>& quat) {
            chrono::ChQuaternion<double> new_quat;
            swap_NED_NWU(new_quat); // TODO : verifier qu'on fait bien le changement
            return new_quat;
        }

        inline FrQuaternion_& swap_NED_NWU(FrQuaternion_ &frQuaternion) {
            frQuaternion.SwapAbsFrameConvention();
            return frQuaternion;
        }

        inline FrQuaternion_ swap_NED_NWU(const FrQuaternion_& frQuaternion) {
            FrQuaternion_ newQuat(frQuaternion);
            swap_NED_NWU(newQuat);
            return newQuat;
        }

        /// Convert a FRyDoM quaternion into a Chrono quaternion
        inline chrono::ChQuaternion<double> Fr2ChQuaternion(const FrQuaternion_& frQuaternion) {
            // TODO : voir si pas conversion plus directe...
            double q0, q1, q2, q3;
            frQuaternion.Get(q0, q1, q2, q3, NWU);
            return chrono::ChQuaternion<double>(q0, q1, q2, q3);
        }

        /// Convert a Chrono quaternion into a FRyDoM quaternion
        inline FrQuaternion_ Ch2FrQuaternion(const chrono::ChQuaternion<double>& chQuaternion) {
            return FrQuaternion_(chQuaternion.e0(), chQuaternion.e1(), chQuaternion.e2(), chQuaternion.e3(), NWU);
        }



    }


}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
