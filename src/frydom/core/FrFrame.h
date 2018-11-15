//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRFRAME_H
#define FRYDOM_FRFRAME_H

#include "chrono/core/ChFrame.h"

#include "FrVector.h"
#include "FrRotation.h"


namespace frydom {

    // Forward declaration
    class FrBody_;
//    class FrTransform_;
    class FrRotation_;
    class FrOffshoreSystem_;





    /// Class defining a frame from a position and a rotation.
    ///
    /// The parent frame is never defined and must be tracked from the context. Indeed, frames may also be thought as
    /// transforms between frames as they are able to locate a "TO" frame with respect to a "FROM" frame.
    /// To get position part of a frame, a frame convention must always be given as argument (NWU/NED) but a frame does
    /// not hold this notion internally as by default, every quantities are always stored in the NWU convention.
    /// Conversions between conventions are done transparently while calling setters and getters. Note that frame
    /// convention does not need to be given when manipulating FrRotation objects as these objects does neither hold the
    /// frame convention notion intenally, conversions are also automatic.
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
        FrFrame_(const Position &pos, const FrQuaternion_& quaternion, FRAME_CONVENTION fc);

        /// Copy Constructor from an other frame
        /// \param otherFrame Frame to be copied
        /// \return the new frame
        FrFrame_& FrFrame(const FrFrame_& otherFrame);

        // TODO : permettre de definir des parametres de Denavit-Hartenberg modifies...


        // Cartesian Position
        /// Set the position in world reference frame of the origin of the present frame, using doubles
        /// \param x X position of the frame
        /// \param y Y position of the frame
        /// \param z Z position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetPosition(double x, double y, double z, FRAME_CONVENTION fc);

        /// Set the position in world reference frame of the origin of the present frame, using a Position
        /// \param position Position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetPosition(const Position& position, FRAME_CONVENTION fc);

        /// Get the position in world reference frame of the origin of the present frame
        /// \param x X position of the frame
        /// \param y Y position of the frame
        /// \param z Z position of the frame
        /// \param fc Frame convention (NED/NWU)
        void GetPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const;

        /// Get the position in world reference frame of the origin of the present frame
        /// \param position Position of the frame
        /// \param fc Frame convention (NED/NWU)
        void GetPosition(Position& position, FRAME_CONVENTION fc) const;

        /// Get the position in world reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the position of the frame
        Position GetPosition(FRAME_CONVENTION fc) const;

        /// Set the X position in world reference frame of the origin of the present frame
        /// \param x X position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetX(double x, FRAME_CONVENTION fc);

        /// Set the Y position in world reference frame of the origin of the present frame
        /// \param y Y position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetY(double y, FRAME_CONVENTION fc);

        /// Set the Z position in world reference frame of the origin of the present frame
        /// \param z Z position of the frame
        /// \param fc Frame convention (NED/NWU)
        void SetZ(double z, FRAME_CONVENTION fc);

        /// Get the X position in world reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the x position of the frame
        double GetX(FRAME_CONVENTION fc) const;

        /// Get the Y position in world reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the y position of the frame
        double GetY(FRAME_CONVENTION fc) const;

        /// Get the Z position in world reference frame of the origin of the present frame
        /// \param fc Frame convention (NED/NWU)
        /// \return  the z position of the frame
        double GetZ(FRAME_CONVENTION fc) const;


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
        void SetRotation(const FrQuaternion_& quaternion);

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
        FrQuaternion_ GetQuaternion() const;

        /// Rotate the present frame, around the X axis of the present frame (if localAxis)
        /// or around the X axis of the world reference frame otherwise, from a value given in radians.
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame x axis (true) or world frame x axis (false)
        void RotX_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the X axis of the present frame (if localAxis)
        /// or around the X axis of the world reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame x axis (true) or world frame x axis (false)
        void RotX_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Y axis of the present frame (if localAxis)
        /// or around the X axis of the world reference frame otherwise, from a value given in radians.
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame y axis (true) or world frame y axis (false)
        void RotY_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Y axis of the present frame (if localAxis)
        /// or around the X axis of the world reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame y axis (true) or world frame y axis (false)
        void RotY_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Z axis of the present frame (if localAxis)
        /// or around the X axis of the world reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in radians
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame z axis (true) or world frame z axis (false)
        void RotZ_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        /// Rotate the present frame, around the Z axis of the present frame (if localAxis)
        /// or around the X axis of the world reference frame otherwise, from a value given in degrees.
        /// \param angle rotation angle in degrees
        /// \param fc frame convention (NED/NWU)
        /// \param localAxis boolean to choose between present frame z axis (true) or world frame z axis (false)
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
        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisFrame(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;

        /// Get the transformation frame, transforming an other frame to this frame
        /// \param otherFrame other frame from which the transformation frame is searched
        /// \param fc frame convention (NED/NWU)
        /// \return the transformation frame
        FrFrame_ GetThisFrameRelativeTransform_WRT_OtherFrame(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;

        /// Inverse a frame transformation
        /// \return the inverse frame transformation
        FrFrame_& Inverse();
        /// Get the inverse of a frame transformation
        /// \return the inverse frame transformation
        FrFrame_ GetInverse() const;


        friend std::ostream&operator<<(std::ostream& os, const FrFrame_& frame);

    private:

        std::ostream& cout(std::ostream& os) const;

        friend class FrInertiaTensor_;  // TODO : voir pourquoi on definit cette amitie... (et voir si on peut retirer !)


    };

    /// Transform between frames which is also a frame.
    using FrTransform = FrFrame_;


    namespace internal {

        inline FrFrame_ Ch2FrFrame(const chrono::ChFrame<double>& chFrame) {  // OK
            auto pos  = ChVectorToVector3d<Position>(chFrame.GetPos());  // In NWU
            auto quat = Ch2FrQuaternion(chFrame.GetRot());  // In NWU
            return FrFrame_(pos, quat, NWU);
        }

        inline chrono::ChFrame<double> Fr2ChFrame(const FrFrame_& frFrame) {
            auto pos = Vector3dToChVector(frFrame.GetPosition(NWU));
            auto quat = Fr2ChQuaternion(frFrame.GetQuaternion());
            return chrono::ChFrame<double>(pos, quat);
        }

    }  // end namespace internal


}  // end namespace frydom



#endif //FRYDOM_FRFRAME_H
