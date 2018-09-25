//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRFRAME_H
#define FRYDOM_FRFRAME_H


#include "chrono/core/ChFrame.h"
#include "FrRotation.h"


namespace frydom {

    // Forward declaration
    class FrBody_;
    class FrTransform_;


    class FrFrame_ {

    private:

        chrono::ChCoordsys<double> m_chronoCoordsys;   ///< The embedded chrono frame // TODO: avoir un _FrFrameBase pour cacher le chrono...

        FrFrame_* m_parentFrame;


    public:

        FrFrame_();

        explicit FrFrame_(FrFrame_* frame);

        FrFrame_(FrFrame_* parentFrame, const Vector3d& pos, const FrRotation_& rotation);

        void SetParentFrame(FrFrame_* parentFrame);

        std::shared_ptr<FrFrame_> NewRelFrame(Vector3d pos, FrRotation_ rot) const;

        std::shared_ptr<FrFrame_> NewRelFrame(Vector3d pos) const;

        std::shared_ptr<FrFrame_> NewRelFrame(FrRotation_ rot) const;





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
