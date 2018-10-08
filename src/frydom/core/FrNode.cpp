//
// Created by frongere on 08/09/17.
//

#include "FrNode.h"
#include "FrBody.h"


namespace frydom {

    FrNode_::FrNode_(frydom::FrBody_ *body) : m_body(body) {
        m_chronoMarker = std::make_shared<chrono::ChMarker>();
        m_chronoMarker->SetBody(body->GetChronoBody().get());



    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position) : FrNode_(body) {

    }

    FrNode_::~FrNode_() = default;

    void FrNode_::SetLocalPosition(const Position &position) {

    }

    void FrNode_::SetLocalPosition(double x, double y, double z) {

    }

    void FrNode_::SetLocalFrame(const FrFrame_ &frame) {

    }


}  // end namespace frydom