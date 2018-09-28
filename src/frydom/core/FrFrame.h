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

        FRAME_CONVENTION m_frameConvention;

        chrono::ChFrame<double> m_chronoFrame;   // It is always in NWU

        friend class FrBody_;

    public:

        explicit FrFrame_(FRAME_CONVENTION fc);

        FrFrame_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc);

        FrFrame_(const Position &pos, const FrQuaternion_& quaternion, FRAME_CONVENTION fc);

        FrFrame_& FrFrame(const FrFrame_& otherFrame);


        // Cartesian Position

        void SetPosition(double x, double y, double z, FRAME_CONVENTION fc);

        void SetPosition(const Position& position);

        void GetPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const;

        void GetPosition(Position& position) const;

        Position GetPosition(FRAME_CONVENTION fc) const;

        void SetX(double x, FRAME_CONVENTION fc);

        void SetY(double y, FRAME_CONVENTION fc);

        void SetZ(double z, FRAME_CONVENTION fc);

        double GetX(FRAME_CONVENTION fc) const;

        double& GetX();

        double GetY(FRAME_CONVENTION fc) const;

        double& GetY();

        double GetZ(FRAME_CONVENTION fc) const;

        double& GetZ();


        // Operations

        FrFrame_ operator*(const FrFrame_& otherFrame) const;

        void operator*=(const FrFrame_& otherFrame);


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

        FrRotation_ GetRotation(FRAME_CONVENTION fc) const;

        FrQuaternion_ GetQuaternion(FRAME_CONVENTION fc) const;

        void RotX_RADIANS(double angle, FRAME_CONVENTION fc);

        void RotX_DEGREES(double angle, FRAME_CONVENTION fc);

        void RotY_RADIANS(double angle, FRAME_CONVENTION fc);

        void RotY_DEGREES(double angle, FRAME_CONVENTION fc);

        void RotZ_RADIANS(double angle, FRAME_CONVENTION fc);

        void RotZ_DEGREES(double angle, FRAME_CONVENTION fc);

        void SetRotX_RADIANS(double angle, FRAME_CONVENTION fc);

        void SetRotX_DEGREES(double angle, FRAME_CONVENTION fc);

        void SetRotY_RADIANS(double angle, FRAME_CONVENTION fc);

        void SetRotY_DEGREES(double angle, FRAME_CONVENTION fc);

        void SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc);

        void SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc);


        // Frame conventions

        FRAME_CONVENTION GetFrameConvention() const;

        FRAME_CONVENTION SwapAbsFrameConvention();

        void SetFrameConvention(FRAME_CONVENTION fc);

        void SetNWU();

        void SetNED();





        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisFrame(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;

        FrFrame_ GetThisFrameRelativeTransform_WRT_OtherFrame(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;

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

        inline FrFrame_ Ch2FrFrame(const chrono::ChFrame<double>& chFrame) {
            auto pos  = ChVectorToVector3d<Position>(chFrame.GetPos());  // In NWU
            auto quat = Ch2FrQuaternion(chFrame.GetRot());  // In NWU

            return FrFrame_(pos, quat, NWU);
        }

        inline chrono::ChFrame<double> Fr2ChFrame(const FrFrame_& frFrame) {
            auto pos = Vector3dToChVector(frFrame.GetPosition(NWU));
            auto quat = Fr2ChQuaternion(frFrame.GetQuaternion(NWU));
            chrono::ChFrame<double>(pos, quat);
        }

        inline void swap_NED_NWU(FrFrame_& frFrame) {
            internal::swap_NED_NWU(frFrame.GetQuaternion());
        }


    }  // end namespace internal


}  // end namespace frydom



#endif //FRYDOM_FRFRAME_H
