//
// Created by lletourn on 22/05/19.
//

#include "FrCGeometrical.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/FrVector.h"

namespace frydom {


  FrCGeometrical::FrCGeometrical(const std::shared_ptr<FrNode> &node) : m_node(node) {

  }

  std::shared_ptr<FrNode> FrCGeometrical::GetNode() const {
    return m_node;
  }

//----------------------------------------------------------------------------------------------------------------------

  FrCPoint::FrCPoint(const std::shared_ptr<FrNode> &node) : FrCGeometrical(node) {}

  Position FrCPoint::GetPositionInWorld(FRAME_CONVENTION fc) const {
    return m_node->GetPositionInWorld(fc);
  }


//----------------------------------------------------------------------------------------------------------------------

  FrCAxis::FrCAxis(const std::shared_ptr<FrNode> &node) : FrCGeometrical(node) {
    m_axis = ZAXIS;
  }

  FrCAxis::FrCAxis(const std::shared_ptr<FrNode> &node, AXISLABEL axis) : FrCGeometrical(node), m_axis(axis) {

  }

  Position FrCAxis::GetOriginInWorld(FRAME_CONVENTION fc) const {
    return m_node->GetPositionInWorld(fc);
  }

  Direction FrCAxis::GetDirectionInWorld(FRAME_CONVENTION fc) const {

    switch (m_axis) {
      case XAXIS:
        return m_node->GetFrameInWorld().GetXAxisInParent(fc);
      case YAXIS:
        return m_node->GetFrameInWorld().GetYAxisInParent(fc);
      case ZAXIS:
        return m_node->GetFrameInWorld().GetZAxisInParent(fc);
      default:
        throw FrException("axis not correctly defined");
    }

  }

  AXISLABEL FrCAxis::GetLabel() const {
    return m_axis;
  }

//----------------------------------------------------------------------------------------------------------------------

  FrCPlane::FrCPlane(const std::shared_ptr<FrNode> &node) : FrCGeometrical(node) {
    m_normale = ZAXIS;
  }

  FrCPlane::FrCPlane(const std::shared_ptr<FrNode> &node, AXISLABEL axis) : FrCGeometrical(node), m_normale(axis) {

  }

  Position FrCPlane::GetOriginInWorld(FRAME_CONVENTION fc) const {
    return m_node->GetPositionInWorld(fc);
  }

  Direction FrCPlane::GetNormaleInWorld(FRAME_CONVENTION fc) const {

    switch (m_normale) {
      case XAXIS:
        return m_node->GetFrameInWorld().GetXAxisInParent(fc);
      case YAXIS:
        return m_node->GetFrameInWorld().GetYAxisInParent(fc);
      case ZAXIS:
        return m_node->GetFrameInWorld().GetZAxisInParent(fc);
      default:
        throw FrException("normal not correctly defined");
    }

  }

  double FrCPlane::GetDistanceToPointInWorld(Position PointInWorld, FRAME_CONVENTION fc) const {
    Position vector = PointInWorld - GetOriginInWorld(fc);

    return vector.dot(GetNormaleInWorld(fc));
  }

  Position FrCPlane::GetIntersectionWithLineInWorld(Position P0, Position P1, FRAME_CONVENTION fc) const {
    auto normale = GetNormaleInWorld(fc);

    // check if P0P1 is parallel to the plan / or P0P1 null
    Direction line = (P1 - P0);
    assert(line.norm() > 1E-16);
    assert(line.dot(normale) != 0);

    // P0O
    Direction vector = GetOriginInWorld(fc) - P0;

    // s_i
    double s = vector.dot(normale) / line.dot(normale);

    // P_i
    Position Intersection = P0 + (P1 - P0) * s;

    return {Intersection.GetX(), Intersection.GetY(), Intersection.GetZ()};
  }

  Position FrCPlane::GetClosestPointOnPlaneInWorld(Position PointInWorld, FRAME_CONVENTION fc) const {
    return GetIntersectionWithLineInWorld(PointInWorld, PointInWorld + GetNormaleInWorld(fc), fc);
  }
} // end namespace frydom