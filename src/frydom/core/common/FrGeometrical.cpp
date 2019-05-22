//
// Created by lletourn on 22/05/19.
//

#include "FrGeometrical.h"
#include "FrNode.h"
#include "frydom/core/math/FrVector.h"

namespace frydom {


    FrGeometrical::FrGeometrical(const std::shared_ptr<FrNode> &node) : m_node(node) {

    }

    std::shared_ptr<FrNode> FrGeometrical::GetNode() const {
        return m_node;
    }

//----------------------------------------------------------------------------------------------------------------------

    FrPoint::FrPoint(const std::shared_ptr<FrNode> &node) : FrGeometrical(node) {}

    Position FrPoint::GetPositionInWorld(FRAME_CONVENTION fc) const {
        return m_node->GetPositionInWorld(fc);
    }


//----------------------------------------------------------------------------------------------------------------------

    FrAxis::FrAxis(const std::shared_ptr<FrNode> &node) : FrGeometrical(node) {
        m_axis = ZAXIS;
    }

    FrAxis::FrAxis(const std::shared_ptr<FrNode> &node, AXISLABEL axis) : FrGeometrical(node), m_axis(axis) {

    }

    Position FrAxis::GetOriginInWorld(FRAME_CONVENTION fc) const {
        return m_node->GetPositionInWorld(fc);
    }

    Direction FrAxis::GetDirectionInWorld(FRAME_CONVENTION fc) const {

        switch (m_axis) {
            case XAXIS:
                return m_node->GetFrameInWorld().GetXAxisInParent(fc);
            case YAXIS:
                return m_node->GetFrameInWorld().GetYAxisInParent(fc);
            case ZAXIS:
                return m_node->GetFrameInWorld().GetZAxisInParent(fc);
        }

    }

//----------------------------------------------------------------------------------------------------------------------

    FrPlane::FrPlane(const std::shared_ptr<FrNode> &node) : FrGeometrical(node) {
        m_normale = ZAXIS;
    }

    FrPlane::FrPlane(const std::shared_ptr<FrNode> &node, AXISLABEL axis) : FrGeometrical(node), m_normale(axis) {

    }

    Position FrPlane::GetOriginInWorld(FRAME_CONVENTION fc) const {
        return m_node->GetPositionInWorld(fc);
    }

    Direction FrPlane::GetNormaleInWorld(FRAME_CONVENTION fc) const {

        switch (m_normale) {
            case XAXIS:
                return m_node->GetFrameInWorld().GetXAxisInParent(fc);
            case YAXIS:
                return m_node->GetFrameInWorld().GetYAxisInParent(fc);
            case ZAXIS:
                return m_node->GetFrameInWorld().GetZAxisInParent(fc);
        }

    }
} // end namespace frydom