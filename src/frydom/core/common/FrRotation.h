// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRROTATION_H
#define FRYDOM_FRROTATION_H


#include "chrono/core/ChQuaternion.h"

#include "FrConvention.h"
#include "frydom/core/math/FrEulerAngles.h"
#include "frydom/core/math/FrVector.h"


namespace frydom {

  /**
  * \class FrUnitQuaternion
  * \brief Class for using quaternion.
  */
  class FrUnitQuaternion {
    /// INFO : This quaternion class is only used in FRyDoM to represent rotations, in contrary to Chrono.
    /// The unit quaternion is then ALWAYS defined normalized.

   private:

    chrono::ChQuaternion<double> m_chronoQuaternion;  ///< Chrono class for quaternion,
    ///< Chrono objects are always stored in NWU frame convention

    /// Get the Chrono quaternion object
    /// \return Chrono quaternion object
    const chrono::ChQuaternion<double> &GetChronoQuaternion() const;

    friend class FrRotation;


   public:

    /// Default constructor.
    /// Note that this constructs a {1,0,0,0} unit quaternion, not a null quaternion {0,0,0,0}.
    FrUnitQuaternion();

    /// Constructor from four doubles. The first is the real part, others are i,j,k imaginary parts
    /// the quaternion represented by {q0,q1,q2,q3} MUST be normalized.
    /// \param q0 real part
    /// \param q1 first imaginary part
    /// \param q2 second imaginary part
    /// \param q3 third imaginary part
    /// \param fc frame convention (NED/NWU)
    FrUnitQuaternion(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc);

    /// Constructor from four doubles. The first is the real part, others are i,j,k imaginary parts
    /// \param q0 real part
    /// \param q1 first imaginary part
    /// \param q2 second imaginary part
    /// \param q3 third imaginary part
    /// \param non_normalized bool to check if the quaternion represented by (q0,q1,q2,q3) is already normalized
    /// \param fc frame convention (NED/NWU)
    FrUnitQuaternion(double q0, double q1, double q2, double q3, bool non_normalized, FRAME_CONVENTION fc);

    /// Constructor from a direction and an angle
    /// \param axis direction of the rotation, MUST be normalized
    /// \param angleRAD angle in radians
    /// \param fc frame convention (NED/NWU)
    FrUnitQuaternion(const Direction &axis, double angleRAD, FRAME_CONVENTION fc);

    /// Copy Constructor from an other quaternion
    /// \param other quaternion copied
    FrUnitQuaternion(const FrUnitQuaternion &other);

    /// Set the quaternion real part and imaginary parts,
    /// the quaternion reprented by {q0,q1,q2,q3} MUST be normalized.
    /// \param q0 real part
    /// \param q1 first imaginary part
    /// \param q2 second imaginary part
    /// \param q3 third imaginary part
    /// \param fc frame convention (NED/NWU)
    void Set(double q0, double q1, double q2, double q3,
             FRAME_CONVENTION fc); // TODO : fusionner avec le methode suivante...

    /// Set the quaternion real part and imaginary parts
    /// \param q0 real part
    /// \param q1 first imaginary part
    /// \param q2 second imaginary part
    /// \param q3 third imaginary part
    /// \param non_normalized bool to check if the quaternion represented by (q0,q1,q2,q3) is already normalized
    /// \param fc frame convention (NED/NWU)
    void Set(double q0, double q1, double q2, double q3, bool non_normalized, FRAME_CONVENTION fc);

    /// Set the quaternion using an other quaternion
    /// \param quaternion quaternion
    void Set(const FrUnitQuaternion &quaternion); // TODO : supprimer, on a deja l'operateur =.

    /// Set the quaternion using a direction and an angle
    /// \param axis direction of the rotation, MUST be normalized
    /// \param angleRAD angle in radians
    /// \param fc frame convention (NED/NWU)
    void Set(const Direction &axis, double angleRAD, FRAME_CONVENTION fc);

    /// Set the quaternion using a roration matrix.
    /// The matrix must be orthogonal
    /// \param matrix the 3x3 rotation matrix
    /// \param fc frame convention (NED/NWU)
    void Set(const mathutils::Matrix33<double> &matrix, FRAME_CONVENTION fc);

    /// Set the Quaternion to the null rotation (ie the unit quaternion: {1,0,0,0})
    void SetNullRotation();

    /// Get the quaternion real and imaginary parts.
    /// \param q0 real part
    /// \param q1 first imaginary part
    /// \param q2 second imaginary part
    /// \param q3 third imaginary part
    /// \param fc frame convention (NED/NWU)
    void
    Get(double &q0, double &q1, double &q2, double &q3, FRAME_CONVENTION fc) const; // TODO : renommer en GetComponents

    /// Get the direction and angle of the rotation, represented by the quaternion
    /// \param axis direction of the rotation
    /// \param angleRAD angle in radians
    /// \param fc frame convention (NED/NWU)
    void Get(Direction &axis, double &angleRAD, FRAME_CONVENTION fc) const;  // TODO : renommer en GetAxisAngle

    /// Get the X axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    /// \param fc frame convention (NED/NWU)
    /// \return X axis
    Direction GetXAxis(FRAME_CONVENTION fc) const;

    /// Get the Y axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    /// \param fc frame convention (NED/NWU)
    /// \return Y axis
    Direction GetYAxis(FRAME_CONVENTION fc) const;

    /// Get the Z axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    /// \param fc frame convention (NED/NWU)
    /// \return Z axis
    Direction GetZAxis(FRAME_CONVENTION fc) const;


    // Operators

    /// Assignment operator: copy from another quaternion.
    /// \param other quaternion to be assigned
    /// \return the quaternion assigned
    FrUnitQuaternion &operator=(const FrUnitQuaternion &other);

    /// Operator for quaternion product: A*B means the typical quaternion product.
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   concatenation of rotations as:
    ///        frame_rotation_2to0 = frame_rotation_1to0 * frame_rotation_2to1
    /// - pay attention to operator low precedence (see C++ precedence rules!)
    /// - quaternion product is not commutative.
    /// \param other quaternion to be multiplied
    /// \return product of quaternions
    FrUnitQuaternion operator*(const FrUnitQuaternion &other) const;
    // TODO : definir un operateur * template pour les vecteurs

    /// Operator for quaternion product and assignment:
    /// A*=B means A'=A*B, with typical quaternion product.
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   post-concatenation of a rotation in a kinematic chain.
    /// - quaternion product is not commutative.
    /// \param other quaternion to be multiplied
    /// \return product of quaternions
    FrUnitQuaternion &operator*=(const FrUnitQuaternion &other);

    /// Quaternions comparison operator.
    /// \param other other FrUnitQuaternion to compare
    /// \return true if FrUnitQuaternion are equals, false otherwise
    bool operator==(const FrUnitQuaternion &other) const;
        bool IsApprox(const FrUnitQuaternion& other, double prec = 1e-8) const;

        bool IsZero(double prec = 1e-8) const;

    /// Rotate a templated vector A, of type Vector, of a rotation,
    /// on the basis of this quaternion: res=p*[0,A]*p'
    /// \tparam Vector template of the vector argument
    /// \param vector vector to be rotated
    /// \param fc frame convention (NED/NWU)
    /// \return rotated vector
    template<class Vector>
        Vector Rotate(const Vector& vector, FRAME_CONVENTION fc) const {  // TODO : voir si on a pas qqch de plus optimise...
      auto vectorTmp = vector;

      if (IsNED(fc)) internal::SwapFrameConvention<Vector>(vectorTmp);

      auto chronoVector = internal::Vector3dToChVector(vectorTmp);

      vectorTmp = internal::ChVectorToVector3d<Vector>(m_chronoQuaternion.Rotate(chronoVector));

      if (IsNED(fc)) internal::SwapFrameConvention<Vector>(vectorTmp);

      return vectorTmp;
    }

    /// Apply rotation before this rotation.
    /// It corresponds to applying a rotation to the parent and must be expressed in the current parent frame
    /// Practically, this is a left rotation composition
    void RotateInParent(const FrUnitQuaternion &leftQuaternion);

    /// Apply rotation after this rotation.
    /// It corresponds to applying a rotation to the target frame and must be expressed in the current target frame
    /// Practically, this is a right rotation composition
    void RotateInFrame(const FrUnitQuaternion &rightQuaternion);

    /// Inverse the quaternion to get its inverse in place (its vectorial part changes sign).
    /// \return the quaternion conjugate in place
    FrUnitQuaternion &Inverse();

    /// Get the inverse of the quaternion
    /// \return the quaternion inverse
    FrUnitQuaternion GetInverse() const;

    /// Get the 3x3 matrix as a rotation matrix corresponding
    /// to the rotation expressed by the quaternion.
    /// \return 3x3 rotation matrix
    mathutils::Matrix33<double> GetRotationMatrix() const;

    /// Get the 3x3 matrix as a rotation matrix corresponding
    /// to the rotation expressed by the quaternion inverse.
    /// \return 3x3 rotation matrix
    mathutils::Matrix33<double> GetInverseRotationMatrix() const;
    // FIXME : les 4 methodes suivantes sont-elles vraiment utiles ??? De maniere generale dans FRyDoM, on ne manipule
    // que rarement directement les matrices de rotation ...

    /// Compute the left multiplication of a 3x3 matrix A, by the 3x3 rotation matrix corresponding
    ///  to the rotation expressed by the quaternion : res = A * R(quat)
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix, solution of A * R(quat)
    mathutils::Matrix33<double> LeftMultiply(const mathutils::Matrix33<double> &matrix) const;

    /// Compute the right multiplication of a 3x3 matrix A, by the 3x3 rotation matrix corresponding
    ///  to the rotation expressed by the quaternion : res = R(quat) * A
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix, solution of R(quat) * A
    mathutils::Matrix33<double> RightMultiply(const mathutils::Matrix33<double> &matrix) const;

    /// Compute the left multiplication of a 3x3 matrix A, by the 3x3 rotation matrix corresponding
    ///  to the rotation expressed by the quaternion inverse : res = A * Rc(quat)
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix, solution of A * Rc(quat)
    mathutils::Matrix33<double> LeftMultiplyInverse(const mathutils::Matrix33<double> &matrix) const;

    /// Compute the right multiplication of a 3x3 matrix A, by the 3x3 rotation matrix corresponding
    ///  to the rotation expressed by the quaternion inverse : res = Rc(quat) * A
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix, solution of Rc(quat) * A
    mathutils::Matrix33<double> RightMultiplyInverse(const mathutils::Matrix33<double> &matrix) const;


    friend std::ostream &operator<<(std::ostream &os, const FrUnitQuaternion &quaternion);


   private:

    /// Normalize the quaternion
    void Normalize();

    /// Compute the euclidean norm of the quaternion, that is its length or magnitude.
    /// \return the norm of the quaternion
    double Norm() const;

    /// Check if the quaternion represents a rotation (ie its norm == 1).
    bool IsRotation() const;

    std::ostream &cout(std::ostream &os) const;

  };

  /*==================================================================================================================
   *
   * FrRotation
   *
   *
   */

  /**
   * \class FrRotation
   * \brief Class for defining a rotation.
   */
  class FrRotation {

   private:

    FrUnitQuaternion m_frQuaternion;  ///< The internal quaternion.

   public:

    /// Default rotation constructor. This is the null rotation
    FrRotation();

    /// Constructor from a quaternion
    /// \param quaternion Unit quaternion representing the rotation
    explicit FrRotation(FrUnitQuaternion quaternion);

    /// Constructor from axis angle rotation (angle in radians)
    /// \param axis direction of the rotation, MUST be normalized
    /// \param angleRAD angle in radians
    /// \param fc frame convention (NED/NWU)
    FrRotation(const Direction &axis, double angleRAD, FRAME_CONVENTION fc);

    FrRotation(const Direction &xaxis, const Direction &yaxis, const Direction &zaxis, FRAME_CONVENTION fc);

    /// Set the null rotation
    void SetNullRotation();


    // Quaternion representation

    /// Set the quaternion of the rotation
    /// \param quat Unit quaternion representing the rotation
    void Set(const FrUnitQuaternion &quat);

    /// Get the quaternion
    /// \return Unit quaternion representing the rotation
    FrUnitQuaternion &GetQuaternion();

    /// Get the quaternion (const)
    /// \return Unit quaternion representing the rotation
    const FrUnitQuaternion &GetQuaternion() const;


    // Axis angle representation

    /// Set the rotation by axis angle representation (angle in radians)
    /// \param axis direction of the rotation, MUST be normalized
    /// \param angleRAD angle in radians
    /// \param fc frame convention (NED/NWU)
    void SetAxisAngle(const Direction &axis, double angleRAD, FRAME_CONVENTION fc);

    /// Get the rotation by axis angle representation (angle in radians)
    /// \param axis direction of the rotation, normalized
    /// \param angleRAD angle in radians
    /// \param fc frame convention (NED/NWU)
    void GetAxisAngle(Direction &axis, double &angleRAD, FRAME_CONVENTION fc) const;

    /// Get the rotation vector, ie the Angle * Axis vector
    mathutils::Vector3d<double> GetRotationVector(FRAME_CONVENTION fc) const;

    /// Get the axis of the rotation
    /// \param axis direction of the rotation, normalized
    /// \param fc frame convention (NED/NWU)
    void GetAxis(Direction &axis, FRAME_CONVENTION fc);

    /// Get the angle in rotation in space (in radians)
    /// \param angle angle in radians
    void GetAngle(double &angle) const;

    /// Get the angle in rotation in space (in radians)
    /// \return angle in radians
    double GetAngle() const;

    /// Get the rotation matrix representation
    /// \return 3x3 rotation matrix
    mathutils::Matrix33<double> GetRotationMatrix() const;

    /// Get the inverse rotation matrix representation
    /// \return 3x3 inverse rotation matrix
    mathutils::Matrix33<double> GetInverseRotationMatrix() const;

    // Matrix representation

    void Set(const Direction &xaxis, const Direction &yaxis, const Direction &zaxis, FRAME_CONVENTION fc);

    // Euler angles representation

    /// Set the euler angles following the prescribed axis sequence (angles in radians)
    /// Only the Cardan seq is implemented for now
    /// \param phi first angle of the Euler sequence
    /// \param theta second angle of the Euler sequence
    /// \param psi third angle of the Euler sequence
    /// \param seq Euler sequence
    /// \param fc frame convention (NED/NWU)
    void SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

    /// Set the euler angles following the prescribed axis sequence (angles in degrees)
    /// Only the Cardan seq is implemented for now
    /// \param phi first angle of the Euler sequence
    /// \param theta second angle of the Euler sequence
    /// \param psi third angle of the Euler sequence
    /// \param seq Euler sequence
    /// \param fc frame convention (NED/NWU)
    void SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

    /// Set the cardan angles (angles in radians)
    /// \param phi roll angle
    /// \param theta pitch angle
    /// \param psi yaw angle
    /// \param fc frame convention (NED/NWU)
    void SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc);

    /// Set the cardan angles (angles in degrees)
    /// \param phi roll angle
    /// \param theta pitch angle
    /// \param psi yaw angle
    /// \param fc frame convention (NED/NWU)
    void SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc);

    /// Get the euler angles following the prescribed axis sequence (angles in radians)
    /// Only the Cardan seq is implemented for now
    /// \param phi first angle of the Euler sequence
    /// \param theta second angle of the Euler sequence
    /// \param psi third angle of the Euler sequence
    /// \param seq Euler sequence
    /// \param fc frame convention (NED/NWU)
    void GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

    /// Get the euler angles following the prescribed axis sequence (angles in degrees)
    /// Only the Cardan seq is implemented for now
    /// \param phi first angle of the Euler sequence
    /// \param theta second angle of the Euler sequence
    /// \param psi third angle of the Euler sequence
    /// \param seq Euler sequence
    /// \param fc frame convention (NED/NWU)
    void GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

    /// Get the cardan angles (angles in radians)
    /// \param phi roll angle
    /// \param theta pitch angle
    /// \param psi yaw angle
    /// \param fc frame convention (NED/NWU)
    void GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const;

    /// Get the cardan angles (angles in degrees)
    /// \param phi roll angle
    /// \param theta pitch angle
    /// \param psi yaw angle
    /// \param fc frame convention (NED/NWU)
    void GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const;

    /// Get the angles of a fixed axis representation (angles in radians)
    /// \param rx
    /// \param ry
    /// \param rz
    /// \param fc frame convention (NED/NWU)
    //             TODO:        NON IMPLEMENTEE
    void GetFixedAxisAngles_RADIANS(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const;

    /// Get the angles of a fixed axis representation (angles in degrees)
    /// \param rx
    /// \param ry
    /// \param rz
    /// \param fc frame convention (NED/NWU)
    //             TODO:        NON IMPLEMENTEE
    void GetFixedAxisAngles_DEGREES(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const;


    // Helpers to build rotations

    /// Apply a rotation to the current rotation by the axis angle representation (angle in radians)
    /// \param axis direction of the rotation, MUST be normalized
    /// \param angle angle in radians
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotAxisAngle_RADIANS(const Direction &axis, double angle, FRAME_CONVENTION fc);

    /// Apply a rotation to the current rotation by the axis angle representation (angle in degrees)
    /// \param axis direction of the rotation, MUST be normalized
    /// \param angle angle in degrees
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotAxisAngle_DEGREES(const Direction &axis, double angle, FRAME_CONVENTION fc);

    /// Apply a rotation around the X axis to the current rotation (angle in radians)
    /// \param angle angle in radians
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotX_RADIANS(double angle, FRAME_CONVENTION fc);

    /// Apply a rotation around the X axis to the current rotation (angle in degrees)
    /// \param angle angle in degrees
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotX_DEGREES(double angle, FRAME_CONVENTION fc);

    /// Apply a rotation around the Y axis to the current rotation (angle in radians)
    /// \param angle angle in radians
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotY_RADIANS(double angle, FRAME_CONVENTION fc);

    /// Apply a rotation around the Y axis to the current rotation (angle in degrees)
    /// \param angle angle in degrees
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotY_DEGREES(double angle, FRAME_CONVENTION fc);

    /// Apply a rotation around the Z axis to the current rotation (angle in radians)
    /// \param angle angle in radians
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotZ_RADIANS(double angle, FRAME_CONVENTION fc);

    /// Apply a rotation around the Z axis to the current rotation (angle in degrees)
    /// \param angle angle in degrees
    /// \param fc frame convention (NED/NWU)
    /// \return the rotation resulting from the combination of both rotations
    FrRotation &RotZ_DEGREES(double angle, FRAME_CONVENTION fc);

    // =============================================================================================================
    // Operators
    // =============================================================================================================

    /// Assign the other rotation to the current rotation
    /// \param other other rotation, to assign
    /// \return assigned rotation
    FrRotation &operator=(const FrRotation &other);

    /// Compose the current rotation with the other. In a matrix form it would be result = this*other.
    /// \param other other rotation, to compose
    /// \return total rotation
    FrRotation operator*(const FrRotation &other) const;

    /// Compose the current rotation with the other inplace. In a matrix form it would be this = this*other.
    /// \param other other rotation, to compose
    /// \return total rotation
    FrRotation &operator*=(const FrRotation &other);

    /// Component-wise comparison operator.
    /// \param other other FrRotation to compare
    /// \return true if FrRotation are equals, false otherwise
    bool operator==(const FrRotation &other) const;

    /// Multiply a matrix by this rotation on the left
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix solution of the multiplication
    mathutils::Matrix33<double> LeftMultiply(const mathutils::Matrix33<double> &matrix) const;

    /// Multiply a matrix by the inverse of this rotation on the left
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix solution of the multiplication
    mathutils::Matrix33<double> LeftMultiplyInverse(const mathutils::Matrix33<double> &matrix) const;

    /// Multiply a matrix by this rotation on the right
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix solution of the multiplication
    mathutils::Matrix33<double> RightMultiply(const mathutils::Matrix33<double> &matrix) const;

    /// Multiply a matrix by the inverse of this rotation on the right
    /// \param matrix 3x3 matrix to be multiplied
    /// \return 3x3 matrix solution of the multiplication
    mathutils::Matrix33<double> RightMultiplyInverse(const mathutils::Matrix33<double> &matrix) const;


    /// Rotate a vector by the current rotation. Templatized method by the type of vector (Position, Velocity... cf FrVector.h)
    /// \tparam Vector template of the vector argument
    /// \param vector vector to be rotated
    /// \param fc frame convention (NED/NWU)
    /// \return rotated vector
    template<class Vector>
        Vector Rotate(const Vector& vector, FRAME_CONVENTION fc) const {
      auto out = m_frQuaternion.Rotate<Vector>(vector, fc);
//            if (IsNED(fc)) internal::SwapFrameConvention<Vector>(out);
      return out;
    }

    /// Apply rotation before this rotation.
    /// It corresponds to applying a rotation to the parent and must be expressed in the current parent frame
    /// Practically, this is a left rotation composition
    void RotateInParent(const FrUnitQuaternion &leftQuaternion);

    /// Apply rotation after this rotation.
    /// It corresponds to applying a rotation to the target frame and must be expressed in the current target frame
    /// Practically, this is a right rotation composition
    void RotateInFrame(const FrUnitQuaternion &rightQuaternion);

    /// Apply rotation before this rotation.
    /// It corresponds to applying a rotation to the parent and must be expressed in the current parent frame
    /// Practically, this is a left rotation composition
    void RotateInParent(const FrRotation &leftRotation);

    /// Apply rotation after this rotation.
    /// It corresponds to applying a rotation to the target frame and must be expressed in the current target frame
    /// Practically, this is a right rotation composition
    void RotateInFrame(const FrRotation &rightRotation);

    /// Get the X axis of the coordinate system obtained by the rotation
    /// \param fc frame convention (NED/NWU)
    /// \return X axis
    Direction GetXAxis(FRAME_CONVENTION fc) const;

    /// Get the Y axis of the coordinate system obtained by the rotation
    /// \param fc frame convention (NED/NWU)
    /// \return Y axis
    Direction GetYAxis(FRAME_CONVENTION fc) const;

    /// Get the Z axis of the coordinate system obtained by the rotation
    /// \param fc frame convention (NED/NWU)
    /// \return Z axis
    Direction GetZAxis(FRAME_CONVENTION fc) const;


    friend std::ostream &operator<<(std::ostream &os, const FrRotation &rotation);


   private:

    std::ostream &cout(std::ostream &os) const;

  };


  namespace internal {

    // Conversion functions between Chrono quaternions and FRyDoM quaternions

    /// Convert a FRyDoM quaternion into a Chrono quaternion
    inline chrono::ChQuaternion<double> Fr2ChQuaternion(const FrUnitQuaternion &frQuaternion) {  // OK
      double q0, q1, q2, q3;
      frQuaternion.Get(q0, q1, q2, q3, NWU);
      return chrono::ChQuaternion<double>(q0, q1, q2, q3);
    }

    /// Convert a Chrono quaternion into a FRyDoM quaternion
    inline FrUnitQuaternion Ch2FrQuaternion(const chrono::ChQuaternion<double> &chQuaternion) {  // OK
      return FrUnitQuaternion(chQuaternion.e0(), chQuaternion.e1(), chQuaternion.e2(), chQuaternion.e3(), NWU);
    }

    /// Swap the frame convention (NED/NWU) of quaternion coefficients
    inline void SwapQuaternionElementsFrameConvention(double &q0, double &q1, double &q2, double &q3) {
      q2 = -q2;
      q3 = -q3;
    }

    /// Swap the frame convention (NED/NWU) of a ChQuaternion
    inline void SwapChQuaternionFrameConvention(chrono::ChQuaternion<double> &quat) {
      quat.e2() = -quat.e2();
      quat.e3() = -quat.e3();
    }

    /// Swap the frame convention (NED/NWU) of a ChQuaternion
    inline chrono::ChQuaternion<double> SwapChQuaternionFrameConvention(const chrono::ChQuaternion<double> &quat) {
      auto quaternion = quat;
      SwapChQuaternionFrameConvention(quaternion);
      return quaternion;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom



#endif //FRYDOM_FRROTATION_H
