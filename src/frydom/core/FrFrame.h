//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRFRAME_H
#define FRYDOM_FRFRAME_H


//#include <chrono/core/ChFrameMoving.h>
//#include "chrono/physics/ChMarker.h"
#include "chrono/core/ChFrame.h"
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


    class FrFrame_;

    namespace internal {

        FrFrame_ Ch2FrFrame(const chrono::ChFrame<double>& chFrame);

        chrono::ChFrame<double> Fr2ChFrame(const FrFrame_& frFrame);

    }


    class FrFrame_ {

    private:

        chrono::ChFrame<double> m_chronoFrame;   ///< The embedded chrono frame // TODO: avoir un _FrFrameBase pour cacher le chrono...

//        FrFrame_* m_parentFrame;

        friend class FrBody_;

    public:

        FrFrame_();

        FrFrame_(const Vector3d &pos, const FrRotation_ &rotation);

        FrFrame_(const Vector3d &pos, const FrQuaternion_& quaternion);

        FrFrame_& FrFrame(const FrFrame_& otherFrame);


        // Position

        void SetPosition(double x, double y, double z);

        void SetPosition(Vector3d position);

        void GetPosition(double& x, double& y, double& z) const;

        void GetPosition(Vector3d& position) const;

        Vector3d GetPosition() const;

        void SetX(double x);

        void SetY(double y);

        void SetZ(double z);

        double GetX() const;

        double& GetX();

        double GetY() const;

        double& GetY();

        double GetZ() const;

        double& GetZ();


        // Rotation

        void SetRotation(const FrRotation_& rotation);

        void SetRotation(const FrQuaternion_& quaternion);

        void SetNoRotation();

        void SetNoTranslation();

        FrRotation_ GetRotation() const;

//        FrRotation_& GetRotation();

        FrQuaternion_ GetQuaternion() const;


        // Operations

        FrFrame_ operator*(const FrFrame_& otherFrame) const;

        void operator*=(const FrFrame_& otherFrame);


        FrFrame_ GetOtherFrameRelativeTransform(const FrFrame_ &otherFrame);


        FrFrame_& Inverse() {
            m_chronoFrame.Invert();
            return *this;
        }

        FrFrame_ GetInverse() {
            return internal::Ch2FrFrame(m_chronoFrame.GetInverse());
        }


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


}  // end namespace frydom



#endif //FRYDOM_FRFRAME_H
