//
// Created by frongere on 27/09/18.
//

#ifndef FRYDOM_FRVECTOR_H
#define FRYDOM_FRVECTOR_H

#include "chrono/core/ChVector.h"
#include "MathUtils/Vector3d.h"

#include "MathUtils/Matrix.h"
//#include "MathUtils/Matrix33.h"

//#include "FrGeographic.h"



namespace frydom {

//    using Vector3d = mathutils::Vector3d<double>;

//    using Matrix33 = mathutils::Matrix33<double>;


//    class Vector3d: public mathutils::Vector3d<double> {
//
//    private:
//        FRAME_CONVENTION m_frameConvention = NWU;
//
//    public:
//
////        Vector3d() : m_frameConvention(NWU), mathutils::Vector3d<double>() {}
//
//        Vector3d(FRAME_CONVENTION fc) : m_frameConvention(fc), mathutils::Vector3d<double>() {}
//
//        Vector3d(Scalar x, Scalar y, Scalar z, FRAME_CONVENTION fc) : mathutils::Vector3d<double>(x, y, z) {}
//
//        // This constructor allows to construct Vector2d from Eigen expressions
//        template <class OtherDerived>
//        explicit Vector3d(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<Scalar, 3, 1>(other) {}
//
//        // This method allows to assign Eigen expressions to Vector3d
//        template <class OtherDerived>
//        Vector3d& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
//            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
//            return *this;
//        }
//
//        FRAME_CONVENTION GetFrameConvention() const {
//            return m_frameConvention;
//        }
//
//        void SetFrameConvention(FRAME_CONVENTION fc) {
//            if (!HasSameConvention(fc)) {
//                SwapFrameConvention();
//                m_frameConvention = fc;
//            }
//        }
//
//        FRAME_CONVENTION SwapFrameConvention() {
//            this->at(1) = -this->at(1);
//            this->at(2) = -this->at(2);
//
//            if (m_frameConvention == NWU) {
//                m_frameConvention = NED;
//            } else {
//                m_frameConvention = NWU;
//            }
//
//            return m_frameConvention;
//        };
//
//        void SetNED() {
//            SetFrameConvention(NED);
//        }
//
//        void SetNWU() {
//            SetFrameConvention(NWU);
//        }
//
//        bool HasSameConvention(FRAME_CONVENTION fc) const {
//            return (m_frameConvention == fc);
//        }
//
//    };


    class Position : public mathutils::Vector3d<double> {

    public:

        Position() : mathutils::Vector3d<double>() {}

        Position(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        explicit Position(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Position& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->mathutils::Vector3d<double>::operator=(other);
            return *this;
        }

        double GetX() const {
            return this->at(0);
        }

        double& GetX() {
            return this->at(0);
        }

        double GetY() const {
            return this->at(1);
        }

        double& GetY() {
            return this->at(1);
        }

        double GetZ() const {
            return this->at(2);
        }

        double& GetZ() {
            return this->at(2);
        }

    };


    class Direction : public mathutils::Vector3d<double> {

    public:

        Direction() : mathutils::Vector3d<double>() {}

        Direction(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        explicit Direction(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Direction& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->mathutils::Vector3d<double>::operator=(other);
            return *this;
        }

        double Getux() const {
            return this->at(0);
        }

        double& Getux() {
            return this->at(0);
        }

        double Getuy() const {
            return this->at(1);
        }

        double& Getuy() {
            return this->at(1);
        }

        double Getuz() const {
            return this->at(2);
        }

        double& Getuz() {
            return this->at(2);
        }

    };


//    class Velocity : public Vector3d {
//
//    public:
//
//        Velocity() : Vector3d() {}
//
//        Velocity(Scalar x, Scalar y, Scalar z) : Vector3d(x, y, z) {}
//
//        // This constructor allows to construct Vector2d from Eigen expressions
//        template <class OtherDerived>
//        explicit Velocity(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<Scalar, 3, 1>(other) {}
//
//        // This method allows to assign Eigen expressions to Vector3d
//        template <class OtherDerived>
//        Velocity& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
//            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
//            return *this;
//        }
//
//        double GetVx() const {
//            return this->at(0);
//        }
//
//        double& GetVx() {
//            return this->at(0);
//        }
//
//        double GetVy() const {
//            return this->at(1);
//        }
//
//        double& GetVy() {
//            return this->at(1);
//        }
//
//        double GetVz() const {
//            return this->at(2);
//        }
//
//        double& GetVz() {
//            return this->at(2);
//        }
//    };
//
//
//    class Acceleration : public Vector3d {
//
//    public:
//
//        Acceleration() : Vector3d() {}
//
//        Acceleration(Scalar x, Scalar y, Scalar z) : Vector3d(x, y, z) {}
//
//        // This constructor allows to construct Vector2d from Eigen expressions
//        template <class OtherDerived>
//        explicit Acceleration(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<Scalar, 3, 1>(other) {}
//
//        // This method allows to assign Eigen expressions to Vector3d
//        template <class OtherDerived>
//        Acceleration& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
//            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
//            return *this;
//        }
//
//        double GetAx() const {
//            return this->at(0);
//        }
//
//        double& GetAx() {
//            return this->at(0);
//        }
//
//        double GetAy() const {
//            return this->at(1);
//        }
//
//        double& GetAy() {
//            return this->at(1);
//        }
//
//        double GetAz() const {
//            return this->at(2);
//        }
//
//        double& GetAz() {
//            return this->at(2);
//        }
//    };




    namespace internal {

        template <class Vector>
        inline Vector& SwapFrameConvention(Vector& vector) {
            vector[1] = -vector[1];
            vector[2] = -vector[2];
            return vector;
        }

        template <class Vector>
        inline Vector SwapFrameConvention(const Vector& vector) {
            Vector out = vector;
            return SwapFrameConvention(vector);
        }

        /// Convert a chrono 3D vector into a mathutils Vector3d
        template <class Vector>
        inline Vector ChVectorToVector3d(const chrono::ChVector<double>& vector) {
            return Vector(vector.x(), vector.y(), vector.z()); // Always gives a vector expressed in NWU
        }

        /// Convert a mathutils Vector3d into a Chrono 3D vector
        inline chrono::ChVector<double> Vector3dToChVector(const mathutils::Vector3d<double>& vector3d) {
            return chrono::ChVector<double>(vector3d[0], vector3d[1], vector3d[2]);
        }

    }  // end namespace internal

}  // end namespace frydom

#endif //FRYDOM_FRVECTOR_H
