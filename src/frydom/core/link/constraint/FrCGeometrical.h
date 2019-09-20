//
// Created by lletourn on 22/05/19.
//

#ifndef FRYDOM_FRGEOMETRICAL_H
#define FRYDOM_FRGEOMETRICAL_H

#include <memory>
#include "frydom/core/common/FrConvention.h"

namespace frydom {

    // Forward declarations
    template<typename OffshoreSystemType>
    class FrNode;

    class Position;

    class Direction;

    enum AXISLABEL {
      XAXIS, YAXIS, ZAXIS
    };


    template<typename OffshoreSystemType>
    class FrCGeometrical {

     protected:

      std::shared_ptr<FrNode<OffshoreSystemType>> m_node;

     public:

      explicit FrCGeometrical(const std::shared_ptr<FrNode<OffshoreSystemType>> &node);

      std::shared_ptr<FrNode<OffshoreSystemType>> GetNode() const;

    };


    template<typename OffshoreSystemType>
    class FrCPoint : public FrCGeometrical<OffshoreSystemType> {

     public:

      explicit FrCPoint(const std::shared_ptr<FrNode<OffshoreSystemType>> &node);

      Position GetPositionInWorld(FRAME_CONVENTION fc) const;

    };

    template<typename OffshoreSystemType>
    class FrCAxis : public FrCGeometrical<OffshoreSystemType> {

     private:

      AXISLABEL m_axis;

     public:

      explicit FrCAxis(const std::shared_ptr<FrNode<OffshoreSystemType>> &node);

      FrCAxis(const std::shared_ptr<FrNode<OffshoreSystemType>> &node, AXISLABEL axis);

      Position GetOriginInWorld(FRAME_CONVENTION fc) const;

      Direction GetDirectionInWorld(FRAME_CONVENTION fc) const;

      AXISLABEL GetLabel() const;;

    };

    template<typename OffshoreSystemType>
    class FrCPlane : public FrCGeometrical<OffshoreSystemType> {

     private:

      AXISLABEL m_normale;

     public:

      explicit FrCPlane(const std::shared_ptr<FrNode<OffshoreSystemType>> &node);

      FrCPlane(const std::shared_ptr<FrNode<OffshoreSystemType>> &node, AXISLABEL axis);

      Position GetOriginInWorld(FRAME_CONVENTION fc) const;

      Direction GetNormaleInWorld(FRAME_CONVENTION fc) const;

      double GetDistanceToPointInWorld(Position PointInWorld, FRAME_CONVENTION fc) const;

      Position GetIntersectionWithLineInWorld(Position P0, Position P1, FRAME_CONVENTION fc) const;

      Position GetClosestPointOnPlaneInWorld(Position PointInWorld, FRAME_CONVENTION fc) const;

    };


} // end namespace frydom


#endif //FRYDOM_FRGEOMETRICAL_H
