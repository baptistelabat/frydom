//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRROTATION_H
#define FRYDOM_FRROTATION_H


#include "chrono/core/ChQuaternion.h"
#include "MathUtils/Vector3d.h"
#include "chrono/core/ChMatrix33.h"

#include "FrEulerAngles.h"



namespace mathutils {
    template <typename Real>
    class Vector3d;
}

namespace frydom {



    // TOTO : mettre ces fonctions dans les utilitaires internes...

    using Vector3d = mathutils::Vector3d<double>;


    // Forward declaration
    class FrQuaternion_;

    namespace internal {

        /// Convert a chrono 3D vector into a mathutils Vector3d
        Vector3d ChVectorToVector3d(const chrono::ChVector<double> vector);

        /// Convert a mathutils Vector3d into a Chrono 3D vector
        chrono::ChVector<double> Vector3dToChVector(const Vector3d vector3d);

        /// Convert a FRyDoM quaternion into a Chrono quaternion
        chrono::ChQuaternion<double> Fr2ChQuaternion(const FrQuaternion_ &frQuaternion);

        /// Convert a Chrono quaternion into a FRyDoM quaternion
        FrQuaternion_ Ch2FrQuaternion(const chrono::ChQuaternion<double> &chQuaternion);

    }



    class FrQuaternion_ {

    private:

        chrono::ChQuaternion<double> m_chronoQuaternion;

        chrono::ChQuaternion<double> GetChronoQuaternion() const;

        friend class FrRotation_;


    public:

        FrQuaternion_();

        FrQuaternion_(double q0, double q1, double q2, double q3);

        FrQuaternion_(double s, Vector3d imag);

        FrQuaternion_(const FrQuaternion_& other);

        void Set(double q0, double q1, double q2, double q3);

        void Set(const FrQuaternion_& quaternion);

        void SetNull();

        void Normalize();

        double Norm() const;

        void SetScalar(double s);

        void SetVector(const Vector3d& v);

        double GetScalar() const;

        Vector3d GetVector() const;

        void Get(double& q0, double& q1, double& q2, double& q3) const;

        // Operators

        FrQuaternion_& operator=(const FrQuaternion_& other);

        FrQuaternion_ operator+() const;
        FrQuaternion_ operator-() const;

        FrQuaternion_ operator*(const FrQuaternion_& other) const;

        FrQuaternion_&operator*=(const FrQuaternion_& other);

        Vector3d Rotate(const Vector3d& vector);

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

        void SetNull();


        // Quaternion representation

        void SetQuaternion(const FrQuaternion_& quat);

        FrQuaternion_& GetQuaternion();

        FrQuaternion_ GetQuaternion() const;


        // Axis angle representation

        void SetAxisAngle(const Vector3d& axis, double angle);

        void GetAxisAngle(Vector3d& axis, double angle);

        void GetAxis(Vector3d& axis);

        void GetAngle(double& angle);


        // Rotation matrix representation

        void SetRotationMatrix(const FrRotationMatrix_& mat);

        FrRotationMatrix_ GetRotationMatrix() const;


        // Euler angles representation

        void SetEulerAngles(double phi, double theta, double psi, EULER_SEQUENCE seq);

        void SetCardanAngles(double phi, double theta, double psi);

        void GetEulerAngles(double& phi, double& theta, double& psi, EULER_SEQUENCE seq) const;

        void GetCardanAngles(double& phi, double& theta, double& psi) const;


        // Operators

        FrRotation_& operator=(const FrRotation_& other);

        FrRotation_ operator*(const FrRotation_& other) const;

        FrRotation_&operator*=(const FrRotation_& other);

        Vector3d Rotate(const Vector3d& vector);


        // TODO : ajouter les GetXAxis etc... cf ChQuaternion pour les methodes...


    };

}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
