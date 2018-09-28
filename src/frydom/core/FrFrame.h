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


//    class FrFrameBase_ : public chrono::ChFrameMoving<double> {
//
////        void essai() {
////            GetPos()
////            GetA_dt()
////            GetA()
////            GetA_dtdt()
////
////            GetInverse()
////            GetPos_dt()
////            GetPos_dtdt()
////
////            GetRot()
////            GetRot_dt()
////            GetWvel_loc()
////            GetWvel_par()
////
////            chrono::ChMarker<double>()
////        }
//    };





    class FrFrame_ {

    private:

        FRAME_CONVENTION m_frameConvention = NWU;

        chrono::ChFrame<double> m_chronoFrame;   ///< The embedded chrono frame // TODO: avoir un _FrFrameBase pour cacher le chrono...

        friend class FrBody_; // So that we don't have to expose chrono API to class user

    public:

        FrFrame_();

        FrFrame_(const Position &pos, const FrRotation_ &rotation);  // TODO : tester que les frames sont consistants...

        FrFrame_(const Position &pos, const FrQuaternion_& quaternion);

        FrFrame_& FrFrame(const FrFrame_& otherFrame);


        // Cartesian Position

        void SetPosition(double x, double y, double z);

        void SetPosition(Position position);

        void GetPosition(double& x, double& y, double& z) const;

        void GetPosition(Position& position) const;

        Position GetPosition() const;

        void SetX(double x);

        void SetY(double y);

        void SetZ(double z);

        double GetX() const;

        double& GetX();

        double GetY() const;

        double& GetY();

        double GetZ() const;

        double& GetZ();

        // Geographic position

        void GetGeographicPosition(double& latitude, double& longitude, double& height) const;

        double GetLatitude() const;

        double GetLongitude() const;

        double GetGeographicHeight() const;

        // Rotation

        void SetRotation(const FrRotation_& rotation);

        void SetRotation(const FrQuaternion_& quaternion);

        void SetNoRotation();

        void SetNoTranslation();

        void SetIdentity();

        FrRotation_ GetRotation() const;

        FrQuaternion_ GetQuaternion() const;

        void RotX_RADIANS(double angle);

        void RotX_DEGREES(double angle);

        void RotY_RADIANS(double angle);

        void RotY_DEGREES(double angle);

        void RotZ_RADIANS(double angle);

        void RotZ_DEGREES(double angle);

        void SetRotX_RADIANS(double angle);

        void SetRotX_DEGREES(double angle);

        void SetRotY_RADIANS(double angle);

        void SetRotY_DEGREES(double angle);

        void SetRotZ_RADIANS(double angle);

        void SetRotZ_DEGREES(double angle);


        // Frame conventions

        FRAME_CONVENTION GetFrameConvention() const;

        FRAME_CONVENTION SwapAbsFrameConvention();

        void SetFrameConvention(FRAME_CONVENTION frameConvention, bool change);

        void SetNWU();

        void SetNED();


        // Operations

        FrFrame_ operator*(const FrFrame_& otherFrame) const;

        void operator*=(const FrFrame_& otherFrame);


        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisFrame(const FrFrame_ &otherFrame) const;

        FrFrame_ GetThisFrameRelativeTransform_WRT_OtherFrame(const FrFrame_ &otherFrame) const;

        FrFrame_& Inverse();

        FrFrame_ GetInverse() const;


        friend std::ostream&operator<<(std::ostream& os, const FrFrame_& frame);

    private:

        std::ostream& cout(std::ostream& os) const;


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
//
//
//    class FrTransform_ : public FrFrame_ {
//
//
//
//    };

    namespace internal {

//        FrFrame_ Ch2FrFrame(const chrono::ChFrame<double>& chFrame);
//
//        chrono::ChFrame<double> Fr2ChFrame(const FrFrame_& frFrame);

        inline FrFrame_ Ch2FrFrame(const chrono::ChFrame<double>& chFrame, FRAME_CONVENTION fc) {
            auto pos = ChVectorToVector3d<Position>(chFrame.GetPos(), fc);
            auto quat = Ch2FrQuaternion(chFrame.GetRot());
            auto frame = FrFrame_(pos, quat);
            frame.SetFrameConvention(fc, false);
            return frame;
        }

        inline chrono::ChFrame<double> Fr2ChFrame(const FrFrame_& frFrame) {
            auto pos = Vector3dToChVector(frFrame.GetPosition());
            auto quat = Fr2ChQuaternion(frFrame.GetQuaternion());
            chrono::ChFrame<double>(pos, quat);
        }

        inline void swap_NED_NWU(FrFrame_& frFrame) {
            internal::swap_NED_NWU(frFrame.GetQuaternion());
        }


    }  // end namespace internal


}  // end namespace frydom



#endif //FRYDOM_FRFRAME_H
