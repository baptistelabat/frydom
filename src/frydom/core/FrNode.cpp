//
// Created by frongere on 08/09/17.
//

#include "FrNode.h"
#include "FrBody.h"


namespace frydom {


    FrNode_::FrNode_(frydom::FrBody_ *body) : m_body(body) {
        auto chronoBody = body->GetChronoBody().get();

        m_chronoMarker = std::make_shared<chrono::ChMarker>();
        m_chronoMarker->SetBody(chronoBody);  //Chrono body can be used because this constructor is a friend of FrBody_

        chronoBody->AddMarker(m_chronoMarker);
    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position) : FrNode_(body) {
        SetLocalPosition(position);
    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position, const FrRotation_ &rotation) : FrNode_(body, position) {
        SetLocalRotation(rotation);
    }

    FrNode_::FrNode_(FrBody_ *body, const Position &position, const FrQuaternion_ &quaternion) : FrNode_(body, position) {
        SetLocalQuaternion(quaternion);
    }

    FrNode_::FrNode_(FrBody_ *body, const FrFrame_ &frame) : FrNode_(body) {
        SetLocalFrame(frame);
    }

    FrNode_::~FrNode_() = default;

    FrBody_* FrNode_::GetBody() {
        return m_body;
    }

    void FrNode_::SetLocalPosition(const Position &position) {
        auto coord = chrono::ChCoordsys<double>(chrono::ChVector<double>(internal::Vector3dToChVector(position)));
        m_chronoMarker->Impose_Rel_Coord(coord);
    }

    void FrNode_::SetLocalPosition(double x, double y, double z) {
        SetLocalPosition(Position(x, y, z));
    }

    void FrNode_::SetLocalQuaternion(const FrQuaternion_ &quaternion) {
        auto coord = chrono::ChCoordsys<double>(chrono::VNULL, internal::Fr2ChQuaternion(quaternion));
        m_chronoMarker->Impose_Rel_Coord(coord);
    }

    void FrNode_::SetLocalRotation(const FrRotation_ &rotation) {
        SetLocalQuaternion(rotation.GetQuaternion());
    }

    void FrNode_::SetLocalFrame(const FrFrame_ &frame) {
        auto chCoord = chrono::ChCoordsys<double>(
                internal::Vector3dToChVector(frame.GetPosition(NWU)),
                internal::Fr2ChQuaternion(frame.GetQuaternion())
                );
        m_chronoMarker->Impose_Rel_Coord(chCoord);
    }

    Position FrNode_::GetAbsPosition() {
        return internal::ChVectorToVector3d<Position>(m_chronoMarker->GetPos());
    }

    void FrNode_::GetAbsPosition(Position &position) {
        position = GetAbsPosition();
    }




    void FrNode_::Initialize() {

    }

    void FrNode_::StepFinalize() {

    }


}  // end namespace frydom