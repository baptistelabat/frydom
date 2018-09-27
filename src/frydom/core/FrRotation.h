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


    class FrQuaternion_ {

    private:

        FRAME_CONVENTION m_frameConvention = NWU;

        chrono::ChQuaternion<double> m_chronoQuaternion;

        chrono::ChQuaternion<double> GetChronoQuaternion() const;

        friend class FrRotation_;


    public:

        FrQuaternion_();

        FrQuaternion_(double q0, double q1, double q2, double q3);

        FrQuaternion_(double s, Direction imag);

        FrQuaternion_(const FrQuaternion_& other);

        void Set(double q0, double q1, double q2, double q3);

        void Set(const FrQuaternion_& quaternion);

        void SetNull();

        void Normalize();

        double Norm() const;

        void SetScalar(double s);

        void SetDirection(const Direction &v);

        double GetScalar() const;

        Direction GetDirection() const;

        void Get(double& q0, double& q1, double& q2, double& q3) const;

        FRAME_CONVENTION GetFrameConvention() const;

        FRAME_CONVENTION SwapAbsFrameConvention();

        void SetFrameConvention(FRAME_CONVENTION frameConvention, bool change=false);

        void SetNWU();

        void SetNED();

//        bool IsAbsolute() const;
//
//        bool IsRelative() const;


        // Operators

        FrQuaternion_& operator=(const FrQuaternion_& other);

        FrQuaternion_ operator+() const;
        FrQuaternion_ operator-() const;

        FrQuaternion_ operator*(const FrQuaternion_& other) const;

        FrQuaternion_&operator*=(const FrQuaternion_& other);

        Position Rotate(const Position& vector);

        FrQuaternion_& Inverse();

        FrQuaternion_ Inverse() const;

    };


    // FIXME : on ne devrait pas avoir besoin d'avoir une telle classe, la classe FrRotation_ suffit...
    class FrRotationMatrix_ {

    private:
        chrono::ChMatrix33<double> m_matrix;

        chrono::ChMatrix33<double> GetChMatrix() const;

        friend class FrRotation_;

    public:
        FrRotationMatrix_();

        FrRotationMatrix_(const FrRotationMatrix_& other);



        // TODO : ajouter des setters et les operations courantes...


    };



    class FrRotation_ {

    private:

        FrQuaternion_ m_quaternion;

    public:

        /// Default rotation constructor. This is the null rotation
        FrRotation_();

        explicit FrRotation_(FrQuaternion_ quaternion);

        void SetNull();


        // Quaternion representation

        void SetQuaternion(const FrQuaternion_& quat);

        FrQuaternion_& GetQuaternion();

        FrQuaternion_ GetQuaternion() const;


        // Frame conventions

        inline FRAME_CONVENTION GetFrameConvention() const;

        FRAME_CONVENTION SwapAbsFrameConvention();

        void SetFrameConvention(FRAME_CONVENTION frameConvention, bool change);

        void SetNWU();

        void SetNED();


        // Axis angle representation

        void SetAxisAngle(const Direction& axis, double angle);

        void GetAxisAngle(Direction& axis, double angle);

        void GetAxis(Direction& axis);

        void GetAngle(double& angle);


        // Rotation matrix representation

        void SetRotationMatrix(const FrRotationMatrix_& mat);

        FrRotationMatrix_ GetRotationMatrix() const;


        // Euler angles representation

        void SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq);
        
        void SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq);

        void SetCardanAngles_RADIANS(double phi, double theta, double psi);

        void SetCardanAngles_DEGREES(double phi, double theta, double psi);

        void GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq) const;

        void GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq) const;

        void GetCardanAngles_RADIANS(double &phi, double &theta, double &psi) const;

        void GetCardanAngles_DEGREES(double &phi, double &theta, double &psi) const;

        void GetFixedAxisAngles_RADIANS(double& rx, double& ry, double& rz) const;

        void GetFixedAxisAngles_DEGREES(double& rx, double& ry, double& rz) const;


        // Helpers to build rotations

        void RotAxisAngle_RADIANS(const Direction& axis, double angle);

        void RotAxisAngle_DEGREES(const Direction& axis, double angle);

        void RotX_RADIANS(double angle);

        void RotX_DEGREES(double angle);

        void RotY_RADIANS(double angle);

        void RotY_DEGREES(double angle);

        void RotZ_RADIANS(double angle);

        void RotZ_DEGREES(double angle);


        // Operators

        FrRotation_& operator=(const FrRotation_& other);

        FrRotation_ operator*(const FrRotation_& other) const;

        FrRotation_&operator*=(const FrRotation_& other);

        Position Rotate(const Position& vector);

        Direction GetXAxis() const;

        Direction GetYAxis() const;

        Direction GetZAxis() const;





        friend std::ostream& operator<<(std::ostream& os, const FrRotation_& rotation);

        // TODO : ajouter les GetXAxis etc... cf ChQuaternion pour les methodes...



    private:

        std::ostream& cout(std::ostream& os) const;

    };



    namespace internal {

        /// Convert a FRyDoM quaternion into a Chrono quaternion
        inline chrono::ChQuaternion<double> Fr2ChQuaternion(const FrQuaternion_& frQuaternion) {
            // TODO : voir si pas conversion plus directe...
            return chrono::ChQuaternion<double>(frQuaternion.GetScalar(), Vector3dToChVector(frQuaternion.GetDirection()));
        }

        /// Convert a Chrono quaternion into a FRyDoM quaternion
        inline FrQuaternion_ Ch2FrQuaternion(const chrono::ChQuaternion<double>& chQuaternion) {
            return FrQuaternion_(chQuaternion.e0(), chQuaternion.e1(), chQuaternion.e2(), chQuaternion.e3());
        }

        inline void swap_NED_NWU(FrQuaternion_ &frQuaternion) {
            frQuaternion.SwapAbsFrameConvention();
        }

        inline FrQuaternion_ swap_NED_NWU(const FrQuaternion_& frQuaternion) {
            FrQuaternion_ newQuat(frQuaternion);
            swap_NED_NWU(newQuat);
            return newQuat;
        }

    }


}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
