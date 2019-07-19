//
// Created by lletourn on 16/07/19.
//

#ifndef FRYDOM_FRPLANE_H
#define FRYDOM_FRPLANE_H

#include <memory>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrFrame.h"

namespace frydom{

    namespace geom {

        class FrPlane {

        public:

            FrPlane(const Position& origin, const Direction& normal, FRAME_CONVENTION fc);

            FrPlane(const std::vector<Position> &cloudPoints, FRAME_CONVENTION fc);

            void SetOrigin(const Position& origin, FRAME_CONVENTION fc);

            Position GetOrigin(FRAME_CONVENTION fc) const;

            void GetOrigin(Position origin, FRAME_CONVENTION fc) const;

            void SetNormal(const Direction &normal, FRAME_CONVENTION fc);

            Direction GetNormal(FRAME_CONVENTION fc) const;

            void GetNormal(Direction normal, FRAME_CONVENTION fc) const;

            FrFrame GetFrame() const;

            double GetDistanceToPoint(Position PointInWorld, FRAME_CONVENTION fc) const;

            double GetSignedDistanceToPoint(Position PointInWorld, FRAME_CONVENTION fc) const;

            Position GetIntersectionWithLine(Position P0, Position P1, FRAME_CONVENTION fc) const;

            Position GetClosestPointOnPlane(Position PointInWorld, FRAME_CONVENTION fc) const;




        private:

            void BuildFrame();

            Position m_origin;          ///< plane origin
            Direction m_normal;         ///< plane normal
            FrFrame m_frame;            ///< plane reference frame

        };

    } //end namespace frydom::mesh

}// end namespace frydom
#endif //FRYDOM_FRPLANE_H
