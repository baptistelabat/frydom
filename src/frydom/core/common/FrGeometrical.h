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

    enum AXISLABEL {XAXIS, YAXIS, ZAXIS};

    class FrGeometrical {

    protected:

        std::shared_ptr<FrNode> m_node;

    public:

        explicit FrGeometrical(const std::shared_ptr<FrNode>& node);

        std::shared_ptr<FrNode> GetNode() const;

    };

    class FrPoint : public FrGeometrical {

    public:

        explicit FrPoint(const std::shared_ptr<FrNode>& node);

        Position GetPositionInWorld(FRAME_CONVENTION fc) const;

    };


    class FrAxis : public FrGeometrical {

    private:

        AXISLABEL m_axis;

    public:

        explicit FrAxis(const std::shared_ptr<FrNode>& node);

        FrAxis(const std::shared_ptr<FrNode>& node, AXISLABEL axis);

        Position GetOriginInWorld(FRAME_CONVENTION fc) const;

        Direction GetDirectionInWorld(FRAME_CONVENTION fc) const;

        AXISLABEL GetLabel() const;;

    };

    class FrPlane : public FrGeometrical {

    private:

        AXISLABEL m_normale;

    public:

        explicit FrPlane(const std::shared_ptr<FrNode>& node);

        FrPlane(const std::shared_ptr<FrNode>& node, AXISLABEL axis);

        Position GetOriginInWorld(FRAME_CONVENTION fc) const;

        Direction GetNormaleInWorld(FRAME_CONVENTION fc) const;


    };


} // end namespace frydom


#endif //FRYDOM_FRGEOMETRICAL_H
