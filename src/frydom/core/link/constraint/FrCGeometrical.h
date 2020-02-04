//
// Created by lletourn on 22/05/19.
//

#ifndef FRYDOM_FRGEOMETRICAL_H
#define FRYDOM_FRGEOMETRICAL_H

#include <memory>
#include "frydom/core/common/FrConvention.h"

namespace frydom {

  // Forward declarations
  class FrNode;

  class Position;

  class Direction;

  enum AXISLABEL {
    XAXIS, YAXIS, ZAXIS
  };

  class FrCGeometrical {

   protected:

    std::shared_ptr<FrNode> m_node;

   public:

    explicit FrCGeometrical(const std::shared_ptr<FrNode> &node);

    std::shared_ptr<FrNode> GetNode() const;

  };

  class FrCPoint : public FrCGeometrical {

   public:

    explicit FrCPoint(const std::shared_ptr<FrNode> &node);

    Position GetPositionInWorld(FRAME_CONVENTION fc) const;

  };


  class FrCAxis : public FrCGeometrical {

   private:

    AXISLABEL m_axis;

   public:

    explicit FrCAxis(const std::shared_ptr<FrNode> &node);

    FrCAxis(const std::shared_ptr<FrNode> &node, AXISLABEL axis);

    Position GetOriginInWorld(FRAME_CONVENTION fc) const;

    Direction GetDirectionInWorld(FRAME_CONVENTION fc) const;

    AXISLABEL GetLabel() const;;

  };

  class FrCPlane : public FrCGeometrical {

   private:

    AXISLABEL m_normale;

   public:

    explicit FrCPlane(const std::shared_ptr<FrNode> &node);

    FrCPlane(const std::shared_ptr<FrNode> &node, AXISLABEL axis);

    Position GetOriginInWorld(FRAME_CONVENTION fc) const;

    Direction GetNormaleInWorld(FRAME_CONVENTION fc) const;

    double GetDistanceToPointInWorld(Position PointInWorld, FRAME_CONVENTION fc) const;

    Position GetIntersectionWithLineInWorld(Position P0, Position P1, FRAME_CONVENTION fc) const;

    Position GetClosestPointOnPlaneInWorld(Position PointInWorld, FRAME_CONVENTION fc) const;

  };


} // end namespace frydom


#endif //FRYDOM_FRGEOMETRICAL_H
