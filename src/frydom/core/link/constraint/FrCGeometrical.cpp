//
// Created by lletourn on 22/05/19.
//

#include "FrCGeometrical.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/FrVector.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrCGeometrical<OffshoreSystemType>::FrCGeometrical(const std::shared_ptr<FrNode<OffshoreSystemType>> &node)
        : m_node(node) {

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrNode<OffshoreSystemType>> FrCGeometrical<OffshoreSystemType>::GetNode() const {
      return m_node;
    }

//----------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrCPoint<OffshoreSystemType>::FrCPoint(const std::shared_ptr<FrNode<OffshoreSystemType>> &node)
        : FrCGeometrical<OffshoreSystemType>(node) {}

    template<typename OffshoreSystemType>
    Position FrCPoint<OffshoreSystemType>::GetPositionInWorld(FRAME_CONVENTION fc) const {
      return this->m_node->GetPositionInWorld(fc);
    }


//----------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrCAxis<OffshoreSystemType>::FrCAxis(const std::shared_ptr<FrNode<OffshoreSystemType>> &node)
        : FrCGeometrical<OffshoreSystemType>(node) {
      m_axis = ZAXIS;
    }

    template<typename OffshoreSystemType>
    FrCAxis<OffshoreSystemType>::FrCAxis(const std::shared_ptr<FrNode<OffshoreSystemType>> &node, AXISLABEL axis)
        : FrCGeometrical<OffshoreSystemType>(node), m_axis(axis) {

    }

    template<typename OffshoreSystemType>
    Position FrCAxis<OffshoreSystemType>::GetOriginInWorld(FRAME_CONVENTION fc) const {
      return this->m_node->GetPositionInWorld(fc);
    }

    template<typename OffshoreSystemType>
    Direction FrCAxis<OffshoreSystemType>::GetDirectionInWorld(FRAME_CONVENTION fc) const {

      switch (m_axis) {
        case XAXIS:
          return this->m_node->GetFrameInWorld().GetXAxisInParent(fc);
        case YAXIS:
          return this->m_node->GetFrameInWorld().GetYAxisInParent(fc);
        case ZAXIS:
          return this->m_node->GetFrameInWorld().GetZAxisInParent(fc);
      }

    }

    template<typename OffshoreSystemType>
    AXISLABEL FrCAxis<OffshoreSystemType>::GetLabel() const {
      return m_axis;
    }

//----------------------------------------------------------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrCPlane<OffshoreSystemType>::FrCPlane(const std::shared_ptr<FrNode<OffshoreSystemType>> &node)
        : FrCGeometrical<OffshoreSystemType>(node) {
      m_normale = ZAXIS;
    }

    template<typename OffshoreSystemType>
    FrCPlane<OffshoreSystemType>::FrCPlane(const std::shared_ptr<FrNode<OffshoreSystemType>> &node, AXISLABEL axis)
        : FrCGeometrical<OffshoreSystemType>(node), m_normale(axis) {

    }

    template<typename OffshoreSystemType>
    Position FrCPlane<OffshoreSystemType>::GetOriginInWorld(FRAME_CONVENTION fc) const {
      return this->m_node->GetPositionInWorld(fc);
    }

    template<typename OffshoreSystemType>
    Direction FrCPlane<OffshoreSystemType>::GetNormaleInWorld(FRAME_CONVENTION fc) const {

      switch (m_normale) {
        case XAXIS:
          return this->m_node->GetFrameInWorld().GetXAxisInParent(fc);
        case YAXIS:
          return this->m_node->GetFrameInWorld().GetYAxisInParent(fc);
        case ZAXIS:
          return this->m_node->GetFrameInWorld().GetZAxisInParent(fc);
      }

    }

    template<typename OffshoreSystemType>
    double FrCPlane<OffshoreSystemType>::GetDistanceToPointInWorld(Position PointInWorld, FRAME_CONVENTION fc) const {
      Position vector = PointInWorld - GetOriginInWorld(fc);

      return vector.dot(GetNormaleInWorld(fc));
    }

    template<typename OffshoreSystemType>
    Position
    FrCPlane<OffshoreSystemType>::GetIntersectionWithLineInWorld(Position P0, Position P1, FRAME_CONVENTION fc) const {
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

    template<typename OffshoreSystemType>
    Position
    FrCPlane<OffshoreSystemType>::GetClosestPointOnPlaneInWorld(Position PointInWorld, FRAME_CONVENTION fc) const {
      return GetIntersectionWithLineInWorld(PointInWorld, PointInWorld + GetNormaleInWorld(fc), fc);
    }
} // end namespace frydom
