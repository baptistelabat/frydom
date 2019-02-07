// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRFRAME_H
#define FRYDOM_FRFRAME_H

#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChFrame.h"

#include "frydom/core/math/FrVector.h"
#include "FrRotation.h"


namespace frydom {

    // Forward declaration
    class FrBody_;
    class FrOffshoreSystem_;





    /// Class defining a frame from a position and a rotation.
    ///
    /// The parent frame is never defined and must be tracked from the context. Indeed, frames may also be thought as
    /// transforms between frames as they are able to locate a "TO" frame with respect to a "FROM" frame.
    /// To get position part of a frame, a frame convention must always be given as argument (NWU/NED) but a frame does
    /// not hold this notion internally as by default, every quantities are always stored in the NWU convention.
    /// Conversions between conventions are done transparently while calling setters and getters. Note that frame
    /// convention does not need to be given when manipulating FrRotation objects as these objects does neither hold the
    /// frame convention notion internally, conversions are also automatic.
    class FrFrame_ {

    private:

        chrono::ChFrame<double> m_chronoFrame;   ///< Chrono objects are always stored in NWU frame convention

    public:

        /// Default constructor that builds a new frame with zero position and rotation
        FrFrame_();

        /// Constructor taking a position and a rotation. Position is given in frame convention
        /// \param pos Position of the frame
        /// \param rotation Rotation of the frame
        /// \param fc Frame convention (NED/NWU)
        FrFrame_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc);

        /// Constructor taking a position and a quaternion. Position is given in frame convention
        /// \param pos Position of the frame
        /// \param quaternion Quaternion of the frame
        /// \param fc Frame convention (NED/NWU)
        FrFrame_(const Position &pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc);

        /// Copy Constructor from an other frame
        /// \param otherFrame Frame to be copied
        /// \return the new frame
        FrFrame_& FrFrame(const FrFrame_& otherFrame);

        // TODO : permettre de definir des parametres de Denavit-Hartenberg modifies...


        // Cartesian Position
        /// Set the position in parent reference frame of the origin of the present frame, using doubles
        /// \param x X position of the frame
        /// \param y Y position of the frame
        /// \param z Z position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetPosition(double x, double y, double z, FRAME_CONVENTION fc);

        /// Set the position in parent reference frame of the origin of the present frame, using a Position
        /// \param position Position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetPosition(const Position& position, FRAME_CONVENTION fc);

        /// Get the position in parent reference frame of the origin of the present frame
        /// \param x X position of the frame
        /// \param y Y position of the frame
        /// \param z Z position of the frame
        /// \param fc Frame convention (NED/NWU)
        void GetPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const;

        /// Get the position in parent reference frame of the origin of the present frame
        /// \param position Position of the frame
        /// \param fc Frame convention (NED/NWU)
        void GetPosition(Position& position, FRAME_CONVENTION fc) const;

        /// Get the position in parent reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the position of the frame
        Position GetPosition(FRAME_CONVENTION fc) const;

        /// Set the X position in parent reference frame of the origin of the present frame
        /// \param x X position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetX(double x, FRAME_CONVENTION fc);

        /// Set the Y position in parent reference frame of the origin of the present frame
        /// \param y Y position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetY(double y, FRAME_CONVENTION fc);

        /// Set the Z position in parent reference frame of the origin of the present frame
        /// \param z Z position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetZ(double z, FRAME_CONVENTION fc);

        /// Get the X position in parent reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the x position of the frame
        double GetX(FRAME_CONVENTION fc) const;

        /// Get the Y position in parent reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the y position of the frame
        double GetY(FRAME_CONVENTION fc) const;

        /// Get the Z position in parent reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the z position of the frame
        double GetZ(FRAME_CONVENTION fc) const;

        /// Set the position and axis in parent reference frame
        /// \param pos Position of the frame
        /// \param e1 direction of the x-axis
        /// \param e2 direction of the y-axis
        /// \param e3 direction of the z-axis
        /// \param fc Frame convention
        void Set(Position pos, Direction e1, Direction e2, Direction e3, FRAME_CONVENTION fc);

        // Operations
        /// The '*' operator transforms a coordinate system, so
        /// transformations can be represented with this syntax:
        ///  new_frame = tr_frame * old_frame;
        /// For a sequence of transformations, i.e. a chain of coordinate
        /// systems, you can also write this (just like you would do with
        /// a sequence of Denavitt-Hartemberg matrix multiplications!)
        ///  new_frame = frame1to0 * frame2to1 * frame3to2 * old_frame;
        /// This operation is not commutative.
        /// \param otherFrame old_frame
        /// \return new_frame
        FrFrame_ operator*(const FrFrame_& otherFrame) const;

        /// Performs post-multiplication of this frame by another
        /// frame, for example: A*=T means  A'=A*T
        /// \param otherFrame T
        void operator*=(const FrFrame_& otherFrame);


        // Rotation

        /// Set the rotation of the present frame, using FrRotation
        /// \param rotation Rotation to be set
        void SetRotation(const FrRotation_& rotation);

        /// Set the rotation of the present frame, using FrQuaternion
        /// \param quaternion Quaternion to be set
        void SetRotation(const FrUnitQuaternion_& quaternion);

        /// Nullify any rotation of the present frame
        void SetNoRotation();

        /// Nullify any translation of the present frame transformation
        void SetNoTranslation();

        /// Set the frame transformation to the Identity transformation
        void SetIdentity();

        /// Get the rotation of the present frame, as FrRotation
        /// \return the rotation of the frame/transformation frame
        FrRotation_ GetRotation() const;

        /// Get the rotation of the present frame, as FrQuaternion
        /// \return the quaternion of the frame/transformation frame
        FrUnitQuaternion_ GetQuaternion() const;

        // FIXME : Du coup en vrai, je ne vois pas l'interet de localAxis ... Retirer ?



        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInFrame(const FrUnitQuaternion_& quaternion);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInFrame(const FrRotation_& rotation);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInFrame(const Direction &direction, double angleRad, FRAME_CONVENTION fc);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInFrame(double phiRad, double thetaRad, double psiRad,  EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInParent(const FrUnitQuaternion_& quaternion);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInParent(const FrRotation_& rotation);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInParent(const Direction &direction, double angleRad, FRAME_CONVENTION fc);

        /// Rotate the present frame around an axis defined in the current frame. This is equivalent to a right
        /// multiplication of the frame current rotation by the given rotation
        void RotateInParent(double phiRad, double thetaRad, double psiRad,  EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        /// Translate the present frame along in its own axes
        void TranslateInFrame(const Translation& translation, FRAME_CONVENTION fc);
        void TranslateInFrame(const Direction& direction, double distance, FRAME_CONVENTION fc);
        void TranslateInFrame(double x, double y, double z, FRAME_CONVENTION fc);
        void TranslateInParent(const Translation& translation, FRAME_CONVENTION fc);
        void TranslateInParent(const Direction& direction, double distance, FRAME_CONVENTION fc);
        void TranslateInParent(double x, double y, double z, FRAME_CONVENTION fc);





        /// Rotate the present frame, around the X axis of the present frame (if localAxis)
        /// or around the X axis of the parent reference frame otherwise, from a value given in radians.
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame x axis (true) or parent frame x axis (false)
        void RotX_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the X axis of the present frame (if localAxis)
        /// or around the X axis of the parent reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame x axis (true) or parent frame x axis (false)
        void RotX_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Y axis of the present frame (if localAxis)
        /// or around the X axis of the parent reference frame otherwise, from a value given in radians.
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame y axis (true) or parent frame y axis (false)
        void RotY_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Y axis of the present frame (if localAxis)
        /// or around the X axis of the parent reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame y axis (true) or parent frame y axis (false)
        void RotY_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Z axis of the present frame (if localAxis)
        /// or around the X axis of the parent reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame z axis (true) or parent frame z axis (false)
        void RotZ_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Z axis of the present frame (if localAxis)
        /// or around the X axis of the parent reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame z axis (true) or parent frame z axis (false)
        void RotZ_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Set the transformation frame to be a rotation around the X axis
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        void SetRotX_RADIANS(double angle, FRAME_CONVENTION fc);

        /// Set the transformation frame to be a rotation around the X axis
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        void SetRotX_DEGREES(double angle, FRAME_CONVENTION fc);

        /// Set the transformation frame to be a rotation around the Y axis
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        void SetRotY_RADIANS(double angle, FRAME_CONVENTION fc);

        /// Set the transformation frame to be a rotation around the Y axis
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        void SetRotY_DEGREES(double angle, FRAME_CONVENTION fc);

        /// Set the transformation frame to be a rotation around the Z axis
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        void SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc);

        /// Set the transformation frame to be a rotation around the Z axis
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        void SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc);

        // TODO : et les angles d'Euler ?

        /// Get the transformation frame, transforming this frame to an other frame
        /// \param otherFrame other frame from which the transformation frame is searched
        /// \param fc frame convention (NED/NWU)
        /// \return the transformation frame
        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisFrame(const FrFrame_ &otherFrame) const;

        /// Get the transformation frame, transforming an other frame to this frame
        /// \param otherFrame other frame from which the transformation frame is searched
        /// \param fc frame convention (NED/NWU)
        /// \return the transformation frame
        FrFrame_ GetThisFrameRelativeTransform_WRT_OtherFrame(const FrFrame_ &otherFrame) const;

        /// Inverse a frame transformation
        /// \return the inverse frame transformation
        FrFrame_& Inverse();

        /// Get the inverse of a frame transformation
        /// \return the inverse frame transformation
        FrFrame_ GetInverse() const;

        /// Get the projection of the current frame into the parent frame XY plane so that the new frame share its z axis
        /// with the parent frame
        /// \return projected frame
        FrFrame_ ProjectToXYPlane(FRAME_CONVENTION fc) const;

        /// Get the frame X axis expressed in the parent frame
        /// \return the X Direction
        Direction GetXAxisInParent(FRAME_CONVENTION fc) const;

        /// Get the frame Y axis expressed in the parent frame
        /// \return the Y Direction
        Direction GetYAxisInParent(FRAME_CONVENTION fc) const;

        /// Get the frame Z axis expressed in the parent frame
        /// \return the Z Direction
        Direction GetZAxisInParent(FRAME_CONVENTION fc) const;


        /// Projects a vector expresseed in parent frame into the current frame coordinates
        /// \return the projected vector
        template <class Vector>
        Vector ProjectVectorParentInFrame(const Vector& parentVector, FRAME_CONVENTION fc) const {  // FIXME : et si le vecteur place en entree est en NED ????
            return GetQuaternion().GetInverse().Rotate<Vector>(parentVector, fc);
        };

        /// Projects a vector expressed in the current frame into the parent frame coordinates
        /// \return the projected vector
        template <class Vector>
        Vector ProjectVectorFrameInParent(const Vector &frameVector, FRAME_CONVENTION fc) const {  // FIXME : et si le vecteur place en entree est en NED ????
            return GetQuaternion().Rotate<Vector>(frameVector, fc);
        }


        friend std::ostream&operator<<(std::ostream& os, const FrFrame_& frame);

    private:

        std::ostream& cout(std::ostream& os) const;

        friend class FrInertiaTensor_;  // TODO : voir pourquoi on definit cette amitie... (et voir si on peut retirer !)


    };

    /// Transform between frames which is also a frame.
    using FrTransform = FrFrame_;


    namespace internal {
        /// Here we define some conversion functions between

        /// Converts a ChFrame into a FrFrame_
        inline FrFrame_ ChFrame2FrFrame(const chrono::ChFrame<double> &chFrame) {  // OK
            return FrFrame_(ChVectorToVector3d<Position>(chFrame.GetPos()),  // In NWU
                            Ch2FrQuaternion(chFrame.GetRot()), // In NWU
                            NWU);
        }

        /// Converts a FrFrame_ into a ChFrame
        inline chrono::ChFrame<double> FrFrame2ChFrame(const FrFrame_ &frFrame) {
            auto pos = Vector3dToChVector(frFrame.GetPosition(NWU));
            auto quat = Fr2ChQuaternion(frFrame.GetQuaternion());
            return chrono::ChFrame<double>(pos, quat);
        }

        /// Converts a FrFrame into a ChCoordSys
        inline chrono::ChCoordsys<double> FrFrame2ChCoordsys(const FrFrame_ &frFrame) {
            auto pos = Vector3dToChVector(frFrame.GetPosition(NWU));
            auto quat = Fr2ChQuaternion(frFrame.GetQuaternion());
            return chrono::ChCoordsys<double>(pos, quat);
        }

        /// Converts a ChCoordsys to a FrFrame
        inline FrFrame_ ChCoordsys2FrFrame(const chrono::ChCoordsys<double> &chCoordsys) {
            return FrFrame_(
                    ChVectorToVector3d<Position>(chCoordsys.pos), // In NWU
                    Ch2FrQuaternion(chCoordsys.rot),
                    NWU
            );
        }


    }  // end namespace internal


}  // end namespace frydom



#endif //FRYDOM_FRFRAME_H
