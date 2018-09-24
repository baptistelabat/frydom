//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRROTATION_H
#define FRYDOM_FRROTATION_H


#include <chrono/core/ChQuaternion.h>
#include <MathUtils/Vector3d.h>
#include <chrono/core/ChMatrix33.h>

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

        inline Vector3d ChVector2Vector3d(chrono::ChVector<double> vector);

        inline chrono::ChVector<double> Vector3d2ChVector(Vector3d vector3d);

        inline chrono::ChQuaternion<double> FrQuaternion2ChQuaternion(const FrQuaternion_& frQuaternion);

        inline FrQuaternion_ ChQuaternion2FrQuaternion(const chrono::ChQuaternion<double>& chQuaternion);

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

    };



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



        FrRotation_();


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

        void SetEulerAngles(double phi, double theta, double psi, EulerSeq seq);

        void SetCardanAngles(double phi, double theta, double psi);

        void GetEulerAngles(double& phi, double& theta, double& psi, EulerSeq seq) const;

        void GetCardanAngles(double& phi, double& theta, double& psi) const;


        // Operators

        FrRotation_& operator=(const FrRotation_& other);

        FrRotation_ operator*(const FrRotation_& other) const;

        FrRotation_&operator*=(const FrRotation_& other);

        Vector3d Rotate(const Vector3d& vector);




    };

}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
