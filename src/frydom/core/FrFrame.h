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
        FrFrame_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc);


        FrFrame_(const Position &pos, const FrQuaternion_& quaternion, FRAME_CONVENTION fc);

        FrFrame_& FrFrame(const FrFrame_& otherFrame);

        // TODO : permettre de definir des parametres de Denavit-Hartenberg modifies...


        // Cartesian Position

        void SetPosition(double x, double y, double z, FRAME_CONVENTION fc);

        void SetPosition(const Position& position, FRAME_CONVENTION fc);

        void GetPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const;

        void GetPosition(Position& position, FRAME_CONVENTION fc) const;

        Position GetPosition(FRAME_CONVENTION fc) const;

        void SetX(double x, FRAME_CONVENTION fc);

        void SetY(double y, FRAME_CONVENTION fc);

        void SetZ(double z, FRAME_CONVENTION fc);

        double GetX(FRAME_CONVENTION fc) const;

        double GetY(FRAME_CONVENTION fc) const;

        double GetZ(FRAME_CONVENTION fc) const;


        // Operations

        FrFrame_ operator*(const FrFrame_& otherFrame) const;

        void operator*=(const FrFrame_& otherFrame);


        // Rotation

        void SetRotation(const FrRotation_& rotation);

        void SetRotation(const FrQuaternion_& quaternion);

        void SetNoRotation();

        void SetNoTranslation();

        void SetIdentity();

        FrRotation_ GetRotation() const;

        FrQuaternion_ GetQuaternion() const;

        void RotX_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        void RotX_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        void RotY_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        void RotY_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        void RotZ_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis);

        void RotZ_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis);

        void SetRotX_RADIANS(double angle, FRAME_CONVENTION fc);

        void SetRotX_DEGREES(double angle, FRAME_CONVENTION fc);

        void SetRotY_RADIANS(double angle, FRAME_CONVENTION fc);

        void SetRotY_DEGREES(double angle, FRAME_CONVENTION fc);

        void SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc);

        void SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc);

        // TODO : et les angles d'Euler ?

        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisFrame(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;

        FrFrame_ GetThisFrameRelativeTransform_WRT_OtherFrame(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;


        // Geographic position

        void GetGeographicPosition(const FrOffshoreSystem_* system, double& latitude, double& longitude, double& height) const;

        double GetLatitude(const FrOffshoreSystem_* system) const;

        double GetLongitude(const FrOffshoreSystem_* system) const;

        double GetGeographicHeight(const FrOffshoreSystem_* system) const;


        FrFrame_& Inverse();

        FrFrame_ GetInverse() const;


        friend std::ostream&operator<<(std::ostream& os, const FrFrame_& frame);

    private:

        std::ostream& cout(std::ostream& os) const;

        friend class FrInertiaTensor_;  // TODO : voir pourquoi on definnit cette amitie... (et voir si on peut retirer !)


        // Node

//        FrNode GetNodeFromMe() const;


//        std::shared_ptr<FrFrame_> NewRelFrame(Vector3d pos, FrRotation_ rot) const;
//
//        std::shared_ptr<FrFrame_> NewRelFrame(Vector3d pos) const;
//
//        std::shared_ptr<FrFrame_> NewRelFrame(FrRotation_ rot) const;





//        std::shared_ptr<FrFrame_> GetParentFrame() const;
//
//        std::shared_ptr<FrBody_> GetBodyOwner() const;

//        inline std::shared_ptr<FrTransform_> GetTransform() const;
//
//
//        void SetAbsPosition(const Vector3d position);
//
//        Vector3d GetAbsPosition() const;




    };

    /// Transoform between frames which is also a frame.
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
