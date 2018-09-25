//
// Created by frongere on 20/09/18.
//

#include "FrFrame.h"

#include "chrono/core/ChMatrixDynamic.h"

#include "FrRotation.h"


namespace frydom {


    FrFrame_::FrFrame_() = default;

    FrFrame_::FrFrame_(frydom::FrFrame_ *frame) : m_chronoCoordsys(), m_parentFrame(frame) {}

    FrFrame_::FrFrame_(FrFrame_* parentFrame, const Vector3d& pos, const FrRotation_& rotation) :
            m_chronoCoordsys(internal::Vector3dToChVector(pos),
                             internal::Fr2ChQuaternion(rotation.GetQuaternion())),
            m_parentFrame(parentFrame) {  // TODO: faire aussi pos et rot

    }

    void FrFrame_::SetParentFrame(frydom::FrFrame_ *parentFrame) {
        m_parentFrame = parentFrame;
    }

    std::shared_ptr<FrFrame_> FrFrame_::NewRelFrame(Vector3d pos, FrRotation_ rot) const {
        auto newFrame = std::make_shared<FrFrame_>();
        // TODO : utiliser la position et la rotation




    }

    std::shared_ptr<FrFrame_> FrFrame_::NewRelFrame(Vector3d pos) const {
        return std::make_shared<FrFrame_>();
    }

    std::shared_ptr<FrFrame_> FrFrame_::NewRelFrame(FrRotation_ rot) const {
        return std::make_shared<FrFrame_>();
    }


}  // end namespace frydom